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
#define ST7789_DC       CONFIG_ST7789_DC_GPIO
#define ST7789_RST      CONFIG_ST7789_RST_GPIO
#define ST7789_BCKL     CONFIG_ST7789_BCKL_GPIO

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
st7789_handle_t st7789_create(const st7789_cfg_t *spi_cfg);
void st7789_send_cmd(uint8_t cmd);
void st7789_send_data(void  *data, uint16_t length);
void st7789_send_color(void  * data, uint16_t length);
void st7789_ready_register_event_cb(st7789_ready_cb_t cb);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* COMPONENTS_ST7789V_ST7789V_HAL_H_ */
