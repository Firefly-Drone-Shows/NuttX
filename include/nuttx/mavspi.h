#pragma once

#include <nuttx/config.h>
#include <stdint.h>

#ifdef CONFIG_MAVSPI

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MIN
  #define MIN(n,m)                  (((n) < (m)) ? (n) : (m))
#endif

#ifndef MAX
  #define MAX(n,m)                  (((n) < (m)) ? (m) : (n))
#endif

/**
 * Register a MAVLink-over-SPI character device.
 *
 * @param path   Device node path (e.g. "/dev/mavspi0")
 * @param spibus SPI bus number (e.g. 6)
 * @param cs     Chip select GPIO
 *
 * @return OK on success, negative errno on failure
 */
int mavspi_register(const char *path, int spibus, uint32_t cs);

//#define MAVSPI_FIFO_SIZE 4096//2048//1024
typedef struct
{
	volatile uint32_t in;
	volatile uint32_t out;
	volatile uint32_t count;
	volatile uint32_t size;
	volatile uint32_t max;
	uint8_t* buffer;
} MAVSPI_FIFO_st;

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MAVSPI */
