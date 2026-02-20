#include <nuttx/config.h>

#ifdef CONFIG_MAVSPI

#include <nuttx/mavspi.h>
#include <nuttx/spi/spi.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/clock.h>

#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <syslog.h>
#include <debug.h>

#include <nuttx/fs/ioctl.h>
#include <termios.h>
#include <poll.h>

#define PX4_SPIDEV_ID(type, index)  ((((type) & 0xffff) << 16) | ((index) & 0xffff))
#define PX4_SPI_DEVICE_ID           (1 << 12)
#define PX4_SPI_DEV_ID(devid)       ((devid) & 0xffff)
#define PX4_SPIDEVID_TYPE(devid)    (((uint32_t)(devid) >> 16) & 0xffff)

FAR struct spi_dev_s *stm32_spibus_initialize(int bus);  // or stm32h7_spibus_initialize()

#define DRV_WIFI_UBLOX 0xE5

#define MAVSPI_STACKSIZE   2048//1024
//#define MAVSPI_PRIORITY    255//SCHED_PRIORITY_FAST_DRIVER//120

#define MAVSPI_SPI_MODE    SPIDEV_MODE0     // set to what your slave expects
#define MAVSPI_SPI_BITS    8
#define MAVSPI_SPI_FREQ    48000000//24000000         // It works with slave RW612 at 48 MHz, keep it at 24 MHz for now //6000000 // 6 MHz clock
#define MAVSPI_PERIOD_US   1000000          // 10ms = 100Hz traffic

#define OP_STATUS          0xA5

#define MAVSPI_NPOLLWAITERS 1

struct mavspi_dev_s {
  FAR struct spi_dev_s *spi;
  uint32_t cs;
  bool stop;
  mutex_t tx_lock;
  mutex_t rx_lock;
  FAR struct pollfd *fds[MAVSPI_NPOLLWAITERS];
  int npoll;
};

uint32_t mavspi_clk_speed = 0;

/*
#define MAVSPI_TX_SIZE (64)
uint8_t mavspi_tx[MAVSPI_TX_SIZE];
uint8_t mavspi_rx[MAVSPI_TX_SIZE];
*/

//uint32_t mavspi_rx_bytes_ready = 0;

// Works with 1024 tx size, MAVSPI_FAST of 20
#define MAVSPI_TX_SIZE 1024 // SPI Slave can only handle 1024 byte transactions
#define MAVSPI_FAST  20   // 0.020 ms delay between complete transactions
#define MAVSPI_MID   20   // 0.010 ms delay in between header and data packet transmissions
#define MAVSPI_SLOW  1000 // 1 ms
uint32_t mavspi_delay = MAVSPI_FAST;
uint32_t mavspi_wait_us = 140; // Use this with debugger to test wait time between transactions

//uint32_t mavspi_tx_bytes = 0;
uint8_t mavspi_tx_2[MAVSPI_TX_SIZE];
uint8_t mavspi_rx_2[MAVSPI_TX_SIZE];

typedef struct
{
	uint32_t state;
	uint32_t master_tx_pend;
	uint32_t master_rx_space;
	uint32_t slave_rx2_space;
	uint32_t slave_tx2_pend;
	uint32_t tx_size;
	uint32_t rx_size;
	uint64_t last_tx;
} SPI_M_st;

SPI_M_st spi_m;

#pragma pack(push, 1)

typedef struct
{
	uint8_t header;
	uint8_t res1;
	uint8_t res2;
	uint8_t res3;
	uint32_t tx_len;
	uint32_t rx_space;
	uint32_t res12;
} MAVSPI_MASTR_HDR_st;
_Static_assert(sizeof(MAVSPI_MASTR_HDR_st) == 16, "ERROR: size of MAVSPI_HEADER_PKT_st is incorrect!");

typedef struct
{
	uint8_t header;
	uint8_t res1;
	uint8_t res2;
	uint8_t res3;
	uint32_t tx_len;
	uint32_t rx_space;
	uint32_t res12;
} MAVSPI_SLAVE_HDR_st;
_Static_assert(sizeof(MAVSPI_SLAVE_HDR_st) == 16, "ERROR: size of MAVSPI_RX_PKT_st is incorrect!");

#pragma pack(pop)

static void mavspi_pollnotify(FAR struct mavspi_dev_s *dev);

// MAVSPI FIFOs
MAVSPI_FIFO_st mavspi_tx_fifo;
MAVSPI_FIFO_st mavspi_rx_fifo;
uint8_t mavspi_tx_buf[16384];
uint8_t mavspi_rx_buf[1024];

uint32_t tx_ovflws_1 = 0;
uint32_t tx_ovflws_2 = 0;
uint64_t tx_disp_time = 0;

static void mavspi_fifo_init(MAVSPI_FIFO_st* fifo, uint8_t* buffer, uint32_t size)
{
	fifo->in = 0;
	fifo->out = 0;
	fifo->count = 0;
	fifo->buffer = buffer;
	fifo->size = size;
	fifo->max = 0;
}

uint32_t mavspi_fifo_free_space(MAVSPI_FIFO_st* fifo)
{
	return fifo->size - fifo->count;
}

uint32_t mavspi_fifo_count(MAVSPI_FIFO_st* fifo)
{
	return fifo->count;
}

bool mavspi_fifo_write(MAVSPI_FIFO_st* fifo, const uint8_t* data, uint32_t length)
{
	if (mavspi_fifo_free_space(fifo) >= length)
	{
		uint32_t chunk1 = fifo->size - fifo->in;
		if (length <= chunk1)
		{
			memcpy(&fifo->buffer[fifo->in], data, length);
		}
		else
		{
			memcpy(&fifo->buffer[fifo->in], data, chunk1);
			memcpy(fifo->buffer, &data[chunk1], length - chunk1);
		}
		fifo->in = (fifo->in + length) % fifo->size;
		fifo->count += length;
		if (fifo->count > fifo->max) fifo->max = fifo->count;
		return true;
	}
	else
	{
		return false;
	}
}

bool mavspi_fifo_read(MAVSPI_FIFO_st* fifo, uint8_t* data, uint32_t length)
{
	if (mavspi_fifo_count(fifo) >= length && data != NULL)
	{
		if (fifo->in > fifo->out)
		{
			memcpy(data, &fifo->buffer[fifo->out], length);
		}
		else
		{
			uint32_t chunk1 = fifo->size - fifo->out;
			if (chunk1 > length)
			{
				chunk1 = length;
				memcpy(data, &fifo->buffer[fifo->out], chunk1);
			}
			else
			{
				memcpy(data, &fifo->buffer[fifo->out], chunk1);
				memcpy(&data[chunk1], fifo->buffer, length - chunk1);
			}
		}
		fifo->out = (fifo->out + length) % fifo->size;
		fifo->count -= length;
		return true;
	}
	else
	{
		return false;
	}
}

static inline void busy_wait_us(uint32_t us)
{
	uint64_t t0 = hrt_absolute_time();
	while (hrt_absolute_time() - t0 < us) { /* spin */ }
}

uint32_t reason = 0;
uint32_t spi_test = 0;
uint32_t spi_test_length = 0;
uint32_t spi_test_sent = 0;
static int mavspi_thread(int argc, FAR char *argv[])
{
  syslog(LOG_INFO, "Test argv1 %s\n", argv[1]);
  FAR struct mavspi_dev_s *dev = (FAR struct mavspi_dev_s *)(uintptr_t)strtoul(argv[1], NULL, 16);

  MAVSPI_MASTR_HDR_st mastr_hdr = {0};
  MAVSPI_SLAVE_HDR_st slave_hdr = {0};

  memset(&spi_m, 0, sizeof(spi_m));

  // Start mavspi_state below
  spi_m.state = 1;

  while (!dev->stop)
  {

    //nxmutex_lock(&dev->lock);
    {
      	//SPI_LOCK(dev->spi, true);
	//mavspi_clk_speed = SPI_SETFREQUENCY(dev->spi, MAVSPI_SPI_FREQ);
	//SPI_SETMODE(dev->spi, MAVSPI_SPI_MODE);
	//SPI_SETBITS(dev->spi, MAVSPI_SPI_BITS);

	// Check state
	//nxmutex_lock(&dev->lock);
	if (spi_m.state == 1)
	{
		volatile uint32_t tx_fifo_cnt = 0;
		volatile uint32_t max = 0;
		volatile uint32_t rx_fifo_free = 0;
		if (spi_test == 0)
		{
			nxmutex_lock(&dev->tx_lock);
			tx_fifo_cnt = mavspi_fifo_count(&mavspi_tx_fifo);
			if (tx_fifo_cnt > MAVSPI_TX_SIZE) tx_fifo_cnt = MAVSPI_TX_SIZE;
			max = mavspi_tx_fifo.max;
			nxmutex_unlock(&dev->tx_lock);
			nxmutex_lock(&dev->rx_lock);
			rx_fifo_free = mavspi_fifo_free_space(&mavspi_rx_fifo);
			if (rx_fifo_free > MAVSPI_TX_SIZE) rx_fifo_free = MAVSPI_TX_SIZE;
			nxmutex_unlock(&dev->rx_lock);
		}

		// DEBUG PRINT
		if (hrt_absolute_time() - tx_disp_time >= 1000000)
		{
			tx_disp_time = hrt_absolute_time();
			syslog(LOG_INFO, "tx1 %lu tx2 %lu max %lu\n", tx_ovflws_1, tx_ovflws_2, max);
		}
		// DEBUG PRINT

		bool doTx = false;
		// Wait for almost full FIFO, that way we don't interrupt mavlink_receiver giving us more bytes
		if (tx_fifo_cnt)// > /*850*/((MAVSPI_TX_SIZE * 7) / 8))
		{
			doTx = true;
			reason = 1;
		}
		else if (hrt_absolute_time() - spi_m.last_tx >= 1000)
		{
			doTx = true;
			reason = 2;
		}

		if (spi_test > 0)
		{
			doTx = true;
			tx_fifo_cnt = MIN(MAVSPI_TX_SIZE, (spi_test_length - spi_test_sent));
			rx_fifo_free = MAVSPI_TX_SIZE;
			mastr_hdr.res1 = 1;
		}

		if (doTx)
		{
			// Send header
			//MAVSPI_MASTR_HDR_st mastr_hdr = {0};
			//MAVSPI_SLAVE_HDR_st slave_hdr = {0};
			mastr_hdr.header = 0x0C;
			// Save this so it does not change before transaction state
			spi_m.master_tx_pend = tx_fifo_cnt;//mavspi_fifo_count(&mavspi_tx_fifo);//mavspi_tx_bytes;
			mastr_hdr.tx_len = spi_m.master_tx_pend;
			// Notify slave how much RX space we have, this is needed for calculation of next packet size
			spi_m.master_rx_space = rx_fifo_free;//mavspi_fifo_free_space(&mavspi_rx_fifo);//1024;
			mastr_hdr.rx_space = spi_m.master_rx_space;

			SPI_LOCK(dev->spi, true);
			SPI_SETFREQUENCY(dev->spi, MAVSPI_SPI_FREQ);
			SPI_SETMODE(dev->spi, MAVSPI_SPI_MODE);
			SPI_SETBITS(dev->spi, MAVSPI_SPI_BITS);
			SPI_SELECT(dev->spi, PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, DRV_WIFI_UBLOX), true);
			SPI_EXCHANGE(dev->spi, (uint8_t*)&mastr_hdr, (uint8_t*)&slave_hdr, sizeof(mastr_hdr));
			SPI_SELECT(dev->spi, PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, DRV_WIFI_UBLOX), false);
			SPI_LOCK(dev->spi, false);

			// Check Rx bytes
			if (slave_hdr.header != 0x0D)
			{
				// Error, did not receive the correct sync byte
				spi_m.state = 1;
				spi_m.slave_tx2_pend = 0;
				spi_m.slave_rx2_space = 0;
				mavspi_delay = MAVSPI_SLOW; // Wait 5 ms before next try
				syslog(LOG_INFO, "Header packet sent, no response\n");
			}
			else
			{
				// Received sync byte header from slave
				spi_m.slave_tx2_pend = slave_hdr.tx_len;
				spi_m.slave_rx2_space = slave_hdr.rx_space;
				if (spi_m.slave_rx2_space > 1024)
				{
					syslog(LOG_INFO, "INVALID slave_rx2_space! %lu \n", spi_m.slave_rx2_space);
				}
				mavspi_delay = MAVSPI_FAST; // Back to 0.5 ms loop speed
				spi_m.last_tx = hrt_absolute_time();

				if (spi_m.slave_tx2_pend > 0)
				{
					// Slave has data to give us, perform next transaction
					spi_m.state = 2;
				}
				else if (spi_m.master_tx_pend >= 1 && spi_m.slave_rx2_space >= 1)
				{
					// We have data to send and slave has enough space for at least 1 byte
					spi_m.state = 2;
				}
				else
				{
					// No data to send, stay in header state for next transaction
					spi_m.state = 1;
					mavspi_delay = MAVSPI_SLOW;
				}

				if (slave_hdr.res1 == 1 && spi_test == 0)
				{
					// Begin spi test
					spi_test = 1;
					spi_test_length = slave_hdr.res12;
					spi_test_sent = 0;
					syslog(LOG_INFO, "Starting SPI test length %lu\n", spi_test_length);
				}
				//syslog(LOG_INFO, "Header packet sent, tx pend %lu, rx pend %lu\n", mavspi_master_tx_pend, mavspi_slave_rx_pend);
			}
		}
		else
		{
			// Not doing a transaction this loop, no TX or not at timeout yet
			mavspi_delay = MAVSPI_SLOW;
			//nxmutex_unlock(&dev->lock);
		}

		if (spi_m.state == 2)
		{
			busy_wait_us(mavspi_wait_us);
			//nxsig_usleep(MAVSPI_MID);
		}
		else
		{
			//nxmutex_unlock(&dev->lock);
		}

	}
	if (spi_m.state == 2)
	{
		// Perform data transaction
		uint32_t next_size = 0;
		// Start with how many bytes to TX, but must not go over slave_tx_space
		if (spi_m.slave_rx2_space < spi_m.master_tx_pend)
		{
			syslog(LOG_INFO, "SPI slave rx space!\n");
		}
		spi_m.tx_size = MIN(spi_m.master_tx_pend, spi_m.slave_rx2_space);
		if (spi_m.tx_size > MAVSPI_TX_SIZE)
		{
			syslog(LOG_INFO, "SPI TX too many!\n");
		}
		// Account for slave pending data, should be at least as big as we can RX that slave has to TX
		spi_m.rx_size = MIN(spi_m.slave_tx2_pend, spi_m.master_rx_space);
		next_size = MAX(spi_m.tx_size, spi_m.rx_size);
		if (next_size > MAVSPI_TX_SIZE)
		{
			syslog(LOG_INFO, "NEXT SIZE too many!\n");
		}

		if (spi_test == 0)
		{
			// Now tx_size is ready, copy bytes from fifo
			nxmutex_lock(&dev->tx_lock);
			mavspi_fifo_read(&mavspi_tx_fifo, mavspi_tx_2, spi_m.tx_size);
			nxmutex_unlock(&dev->tx_lock);
		}
		else
		{
			// Do not copy shit
			spi_test_sent += spi_m.tx_size;
		}

		SPI_LOCK(dev->spi, true);
		SPI_SETFREQUENCY(dev->spi, MAVSPI_SPI_FREQ);
		SPI_SETMODE(dev->spi, MAVSPI_SPI_MODE);
		SPI_SETBITS(dev->spi, MAVSPI_SPI_BITS);
		SPI_SELECT(dev->spi, PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, DRV_WIFI_UBLOX), true);
		SPI_EXCHANGE(dev->spi, mavspi_tx_2, mavspi_rx_2, next_size);
		SPI_SELECT(dev->spi, PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, DRV_WIFI_UBLOX), false);
		SPI_LOCK(dev->spi, false);

		//syslog(LOG_INFO, "Size %" PRIu32 ", sent %lu received %lu\n", next_size, spi_m.tx_size, spi_m.slave_tx2_pend);

		// TODO deliver RX bytes to FIFO
		// rx_size bytes are in mavspi_rx_2!
		//mavspi_rx_bytes_ready = spi_m.rx_size;
		if (spi_m.rx_size > 0 && spi_test == 0)
		{
			//syslog(LOG_INFO, "SPI Rx %lu bytes from ublox\n", mavspi_rx_bytes_ready);
			nxmutex_lock(&dev->rx_lock);
			mavspi_fifo_write(&mavspi_rx_fifo, mavspi_rx_2, spi_m.rx_size);
			mavspi_pollnotify(dev);
			nxmutex_unlock(&dev->rx_lock);
			syslog(LOG_INFO, "SPI RX received %lu bytes\n", spi_m.rx_size);
		}

		//nxmutex_unlock(&dev->lock);

		if (spi_test > 0 && spi_test_sent >= spi_test_length)
		{
			syslog(LOG_INFO, "SPI test complete, sent %lu bytes\n", spi_test_sent);
			spi_test = 0;
		}

		// Now back to header state
		spi_m.state = 1;
		mavspi_delay = MAVSPI_FAST; // Back to 0.5 ms loop speed
	}
	else
	{
		/*
		SPI_LOCK(dev->spi, true);
		SPI_SELECT(dev->spi, PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, DRV_WIFI_UBLOX), true);
		SPI_EXCHANGE(dev->spi, mavspi_tx_2, mavspi_rx_2, mavspi_tx_bytes);
		SPI_SELECT(dev->spi, PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, DRV_WIFI_UBLOX), false);
		SPI_LOCK(dev->spi, false);
		*/
	}

	//syslog(LOG_INFO, "Sent %" PRIu32 " bytes, received %d %d %d %d\n", mavspi_tx_bytes, mavspi_rx_2[0], mavspi_rx_2[1], mavspi_rx_2[2], mavspi_rx_2[3]);

	//mavspi_tx_bytes = 0;
    }
    //nxmutex_unlock(&dev->lock);

    if (mavspi_delay == MAVSPI_SLOW)
    {
      nxsig_usleep(mavspi_delay); // 1 ms, 1000 Hz loop for now
    }
    else
    {
      busy_wait_us(mavspi_wait_us);
    }

  }

  (void)argc;
  (void)argv;
  return 0;
}

// Minimal file ops so register_driver succeeds (not used yet)
static int mavspi_open(FAR struct file *filep);//  { filep->f_priv = filep->f_inode->i_private; return OK; }
static int mavspi_close(FAR struct file *filep) { return OK; }
static ssize_t mavspi_read(FAR struct file *filep, FAR char *b, size_t buflen);// { return 0; }
static ssize_t mavspi_write(FAR struct file *filep, FAR const char *b, size_t buflen);// { return (ssize_t)l; }
static int mavspi_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int mavspi_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup);

static const struct file_operations g_mavspi_fops = {
  mavspi_open,      /* open */
  mavspi_close,     /* close */
  mavspi_read,      /* read */
  mavspi_write,     /* write */
  NULL,             /* seek */
  mavspi_ioctl,     /* ioctl */
  mavspi_poll       /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL            /* unlink */
#endif
};

int mavspi_register(const char *path, int spibus, uint32_t cs)
{
  FAR struct mavspi_dev_s *dev = kmm_zalloc(sizeof(*dev));
  if (!dev) return -ENOMEM;

  dev->cs = cs;

  dev->spi = stm32_spibus_initialize(spibus);
  if (!dev->spi) {
    kmm_free(dev);
    return -ENODEV;
  }

  // Configure SPI hardware
  SPI_LOCK(dev->spi, true);
  mavspi_clk_speed = SPI_SETFREQUENCY(dev->spi, MAVSPI_SPI_FREQ);
  SPI_SETMODE(dev->spi, MAVSPI_SPI_MODE);
  SPI_SETBITS(dev->spi, MAVSPI_SPI_BITS);
  SPI_LOCK(dev->spi, false);

  syslog(LOG_INFO, "MAVSPI clock speed: %" PRIu32 "\n", mavspi_clk_speed);

  for (int i = 0; i < MAVSPI_NPOLLWAITERS; i++)
  {
    // Zero out fds?
    //memset(dev->fds[i], 0, sizeof(FAR struct pollfd));
  }

  int ret = register_driver(path, &g_mavspi_fops, 0666, dev);
  if (ret < 0)
  {
    kmm_free(dev);
    return ret;
  }

  nxmutex_init(&dev->tx_lock);
  nxmutex_init(&dev->rx_lock);

  //mavspi_tx_bytes = 0;
  memset(mavspi_tx_2, 0, sizeof(mavspi_tx_2));
  memset(mavspi_rx_2, 0, sizeof(mavspi_rx_2));

  mavspi_fifo_init(&mavspi_tx_fifo, mavspi_tx_buf, sizeof(mavspi_tx_buf));
  mavspi_fifo_init(&mavspi_rx_fifo, mavspi_rx_buf, sizeof(mavspi_rx_buf));

  // Start worker thread
  FAR char *argv[2];
  char arg0[32];
  snprintf(arg0, sizeof(arg0), "%p", dev);
  argv[0] = arg0;
  argv[1] = NULL;

  // SCHED_PRIORITY_MAX - 15 is what PX4_WQ_HP_BASE uses, copy that
  int priority = sched_get_priority_max(SCHED_FIFO);// - 15;
  pid_t tid = task_create("mavspi", priority, MAVSPI_STACKSIZE, mavspi_thread, argv);
  if (tid < 0)
  {
    unregister_driver(path);
    kmm_free(argv);
    kmm_free(dev);
    return -errno;
  }

  // NOTE: we intentionally do NOT free argv here, because the task uses it.
  // For bringup, leaking this tiny allocation is fine; you can clean it later.
  return OK;
}

static int mavspi_open(FAR struct file *filep)
{
  //  { filep->f_priv = filep->f_inode->i_private; return OK; }
  filep->f_priv = filep->f_inode->i_private;

  syslog(LOG_INFO, "MAVSPI mavspi_open() called!\n");

  return OK;
}

static int mavspi_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct mavspi_dev_s *dev = (FAR struct mavspi_dev_s *)filep->f_priv;
  if (!dev) return -EINVAL;

  switch (cmd)
  {
    // PX4 will try to configure baud/parity/etc via termios
    case TCGETS:
      if (arg)
      {
        // Return a zeroed termios is fine for bring-up
        memset((FAR void *)(uintptr_t)arg, 0, sizeof(struct termios));
      }
      return OK;

    case TCSETS:
    case TCSETSW:
    case TCSETSF:
      // Accept and ignore for now (SPI doesn't have baud in the same way)
      return OK;

    case FIONSPACE:
      if (arg)
      {
	nxmutex_lock(&dev->tx_lock);
	int free = mavspi_fifo_free_space(&mavspi_tx_fifo);
	*(FAR int *)(uintptr_t)arg = free; // or your TX ring free space later
	nxmutex_unlock(&dev->tx_lock);
	if (free < 110)//MAVSPI_FIFO_SIZE)
	{
		tx_ovflws_2++;
		//syslog(LOG_INFO, "TX FIFO NOT EMPTY\n");
	}
      }
      return OK;
    case FIONREAD:
      if (arg)
      {
	nxmutex_lock(&dev->rx_lock);
        // If you have an RX ring, return its used count.
        *(FAR int *)(uintptr_t)arg = (int)mavspi_fifo_count(&mavspi_rx_fifo);//(int)mavspi_rx_bytes_ready;
	nxmutex_unlock(&dev->rx_lock);
      }
      return OK;

    default:
      // Bring-up: don't fail unknown serial ioctls
      return OK;
  }
}

static ssize_t mavspi_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  ssize_t ret = 0;

  FAR struct mavspi_dev_s *dev = (FAR struct mavspi_dev_s *)filep->f_priv;
  if (!dev) return -EINVAL;

  nxmutex_lock(&dev->tx_lock);
  if (buflen <= mavspi_fifo_free_space(&mavspi_tx_fifo))
  {
    //memcpy(mavspi_tx_2, buffer, buflen);
    //mavspi_tx_bytes = buflen;
    mavspi_fifo_write(&mavspi_tx_fifo, (const uint8_t*)buffer, buflen);
    ret = buflen;
    //syslog(LOG_INFO, "MAVSPI %d bytes written\n", buflen);
    // Hack to get mavlink to read faster
    //mavspi_pollnotify(dev);
  }
  else
  {
    //mavspi_tx_bytes = 0;
    //syslog(LOG_ERR, "Too many bytes for MAVSPI: %d\n", buflen);
    //uint32_t tx_ovflws = 0;
    //uint32_t tx_disp_time = 0;
    tx_ovflws_1++;
  }
  nxmutex_unlock(&dev->tx_lock);

  // Return number of bytes written
  return ret;
}

static ssize_t mavspi_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  if (!buffer || buflen == 0)
  {
    return 0;
  }

  FAR struct mavspi_dev_s *dev = (FAR struct mavspi_dev_s *)filep->f_priv;
  if (!dev) return -EINVAL;

  nxmutex_lock(&dev->rx_lock);
  // Return if no data available
  uint32_t rx_ready = mavspi_fifo_count(&mavspi_rx_fifo);
  //if (mavspi_rx_bytes_ready == 0)
  if (rx_ready == 0)
  {
    nxmutex_unlock(&dev->rx_lock);
    return -EAGAIN;
  }

  size_t n = (rx_ready < buflen) ? rx_ready : buflen;

  //memcpy(buffer, mavspi_rx_2, n);
  mavspi_fifo_read(&mavspi_rx_fifo, (uint8_t*)buffer, n);

  nxmutex_unlock(&dev->rx_lock);
  return (ssize_t)n;
}

static int mavspi_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup)
{
  FAR struct mavspi_dev_s *dev = (FAR struct mavspi_dev_s *)filep->f_priv;
  if (!dev || !fds) return -EINVAL;

  //nxmutex_lock(&dev->lock);

  if (setup) {
    if (dev->npoll >= MAVSPI_NPOLLWAITERS) {
      //nxmutex_unlock(&dev->lock);
      return -ENOMEM;
    }
    dev->fds[dev->npoll++] = fds;

    // If data already available, notify immediately
    //if (mavspi_rx_bytes_ready > 0) {
    /*
    if (mavspi_fifo_count(&mavspi_rx_fifo) > 0) {
      fds->revents |= (fds->events & POLLIN);
    }
      */
  } else {
    for (int i = 0; i < dev->npoll; i++) {
      if (dev->fds[i] == fds) {
        dev->fds[i] = dev->fds[dev->npoll - 1];
        dev->fds[dev->npoll - 1] = NULL;
        dev->npoll--;
        break;
      }
    }
  }

  //nxmutex_unlock(&dev->lock);
  return OK;
}

static void mavspi_pollnotify(FAR struct mavspi_dev_s *dev)
{
    for (int i = 0; i < MAVSPI_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = dev->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
	  //syslog(LOG_INFO, "Report events: %08" PRIx32 "\n", fds->revents);
          nxsem_post(fds->sem);
        }
    }
}

#endif // CONFIG_MAVSPI
