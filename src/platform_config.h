#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

/******************************************************************************
 * Platform Detection and Configuration
 *
 * Supported platforms:
 * - PLATFORM_STM32F4: STM32F4xx series
 * - PLATFORM_STM32F1: STM32F1xx series
 * - PLATFORM_ESP32: ESP32 series
 * - PLATFORM_LINUX: Linux platform (for testing/simulation)
 *
 * Define one of these macros in CMakeLists.txt or compiler flags
 *****************************************************************************/

/* Platform auto-detection */
#if defined(STM32F4) || defined(STM32F405xx) || defined(STM32F407xx)
#define PLATFORM_STM32F4
#elif defined(STM32F1) || defined(STM32F103xB) || defined(STM32F103xE)
#define PLATFORM_STM32F1
#elif defined(ESP32) || defined(ESP_PLATFORM)
#define PLATFORM_ESP32
#elif defined(__linux__) || defined(__unix__)
#define PLATFORM_LINUX
#endif

/* Verify platform is defined */
#if !defined(PLATFORM_STM32F4) && !defined(PLATFORM_STM32F1) && !defined(PLATFORM_ESP32) && !defined(PLATFORM_LINUX)
#warning "No platform defined, defaulting to PLATFORM_LINUX"
#define PLATFORM_LINUX
#endif

/* Platform-specific includes */
#ifdef PLATFORM_STM32F4
#include "stm32f4xx_hal.h"
#define UART_INSTANCE USART1
#define SPI_INSTANCE SPI1
#define I2C_INSTANCE I2C1
#define CAN_INSTANCE CAN1
#define ETH_INSTANCE ETH
#endif

#ifdef PLATFORM_STM32F1
#include "stm32f1xx_hal.h"
#define UART_INSTANCE USART1
#define SPI_INSTANCE SPI1
#define I2C_INSTANCE I2C1
#define CAN_INSTANCE CAN1
#define ETH_INSTANCE ETH
#endif

#ifdef PLATFORM_ESP32
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/twai.h"
#include "driver/uart.h"
#include "esp_eth.h"
#define UART_NUM_DEFAULT UART_NUM_1
#define SPI_HOST_DEFAULT SPI2_HOST
#define I2C_NUM_DEFAULT I2C_NUM_0
#endif

#ifdef PLATFORM_LINUX
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <net/if_arp.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#endif

#endif // PLATFORM_CONFIG_H
