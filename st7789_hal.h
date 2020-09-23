/*
 * st7789v_hal.h
 *
 *  Created on: 23 wrz 2020
 *      Author: kurza
 */

#ifndef ST7789_HAL_H_
#define ST7789_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>
#include <stdint.h>

#include <driver/spi_master.h>
#include <driver/gpio.h>

/*********************
 *      DEFINES
 *********************/
#define ST7789_DC       CONFIG_LV_DISP_PIN_DC
#define ST7789_RST      CONFIG_LV_DISP_PIN_RST
#define ST7789_BCKL     CONFIG_LV_DISP_PIN_BCKL

#define ST7789_ENABLE_BACKLIGHT_CONTROL CONFIG_LV_ENABLE_BACKLIGHT_CONTROL

#if CONFIG_LV_BACKLIGHT_ACTIVE_LVL
  #define ST7789_BCKL_ACTIVE_LVL 1
#else
  #define ST7789_BCKL_ACTIVE_LVL 0
#endif

/**********************
 *      TYPEDEFS
 **********************/
typedef struct
{
	spi_host_device_t spi_host;
	gpio_num_t mosi;
	gpio_num_t miso;
	gpio_num_t sclk;
	gpio_num_t cs;
	uint32_t spi_clock_speed_hz;
	gpio_num_t dc;
	uint8_t dma_channel;
} st7789_cfg_t;

typedef void *st7789_handle_t;
typedef void (*st7789_ready_cb_t)(void);

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* COMPONENTS_ST7789V_ST7789V_HAL_H_ */
