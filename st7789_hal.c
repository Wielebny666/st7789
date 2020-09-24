/*
 * st7789v_hal.c
 *
 *  Created on: 23 wrz 2020
 *      Author: kurza
 */


/*********************
 *      INCLUDES
 *********************/
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "esp_log.h"
#include "esp_err.h"

#include "spi_master_ext.h"
#include "st7789_hal.h"
#include "st7789_defs.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct
{
	uint8_t cmd;
	uint8_t data[16];
	uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef struct
{
	spi_device_handle_t spi_handle;
	st7789_cfg_t spi_config;
} st7789_dev_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void IRAM_ATTR spi_pre_transfer_callback(spi_transaction_t *t);
static void IRAM_ATTR spi_ready(spi_transaction_t *trans);
static void st7789_set_orientation(uint8_t orientation);

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "st7789_hal";

static spi_device_handle_t st7789_spi_handle = NULL;
static SemaphoreHandle_t st7789_trans_in_progress = NULL;
static st7789_ready_cb_t IRAM_ATTR st7789_ready_cb;
static volatile bool spi_color_sent;

// @formatter:off
lcd_init_cmd_t st7789_init_cmds[] = {
        {0xCF, {0x00, 0x83, 0X30}, 3},
        {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
        {ST7789_PWCTRL2, {0x85, 0x01, 0x79}, 3},
        {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
        {0xF7, {0x20}, 1},
        {0xEA, {0x00, 0x00}, 2},
        {ST7789_LCMCTRL, {0x26}, 1},
        {ST7789_IDSET, {0x11}, 1},
        {ST7789_VCMOFSET, {0x35, 0x3E}, 2},
        {ST7789_CABCCTRL, {0xBE}, 1},
        {ST7789_MADCTL, {0x00}, 1}, // Set to 0x28 if your display is flipped
        {ST7789_COLMOD, {0x55}, 1},
		{ST7789_INVON, {0}, 0},
        {ST7789_RGBCTRL, {0x00, 0x1B}, 2},
        {0xF2, {0x08}, 1},
        {ST7789_GAMSET, {0x01}, 1},
        {ST7789_PVGAMCTRL, {0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x32, 0x44, 0x42, 0x06, 0x0E, 0x12, 0x14, 0x17}, 14},
        {ST7789_NVGAMCTRL, {0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x31, 0x54, 0x47, 0x0E, 0x1C, 0x17, 0x1B, 0x1E}, 14},
        {ST7789_CASET, {0x00, 0x00, 0x00, 0xEF}, 4},
        {ST7789_RASET, {0x00, 0x00, 0x01, 0x3f}, 4},
        {ST7789_RAMWR, {0}, 0},
        {ST7789_GCTRL, {0x07}, 1},
        {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
        {ST7789_SLPOUT, {0}, 0x80},
        {ST7789_DISPON, {0}, 0x80},
        {0, {0}, 0xff},
    };
// @formatter:on

/**********************
 *      MACROS
 **********************/
#define CHECK(a, ret_val, str, ...)                                           \
    if (!(a))                                                                 \
    {                                                                         \
        ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val);                                                     \
    }

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
st7789_handle_t st7789_create(const st7789_cfg_t *spi_cfg)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	if(st7789_trans_in_progress == NULL)
		st7789_trans_in_progress = xSemaphoreCreateBinary();// xSemaphoreCreateMutex();

	if (st7789_trans_in_progress == NULL)
	{
		ESP_LOGE(TAG, "Create semaphore fail");
		return NULL;
	}
	vQueueAddToRegistry(st7789_trans_in_progress, "st7789_trans_semaphore");
	xSemaphoreGive(st7789_trans_in_progress);

	st7789_dev_t *st7789_dev = (st7789_dev_t*) calloc(1, sizeof(st7789_dev_t));
	CHECK((st7789_dev != NULL), NULL, "ST7789 ALLOC FAIL");

	//Initialize non-SPI GPIOs
	gpio_pad_select_gpio(spi_cfg->dc);
	gpio_set_direction(spi_cfg->dc, GPIO_MODE_OUTPUT);

	spi_bus_config_t buscfg =
		{
			.miso_io_num = spi_cfg->miso,
			.mosi_io_num = spi_cfg->mosi,
			.sclk_io_num = spi_cfg->sclk,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
			.max_transfer_sz = 12800 * 6, // Should keep it large !
			.intr_flags = ESP_INTR_FLAG_IRAM, };

	spi_device_interface_config_t devcfg =
		{
			.clock_speed_hz = spi_cfg->spi_clock_speed_hz / 5, // Clock out frequency
			.mode = 0,                 			  	// SPI mode 0
			.spics_io_num = spi_cfg->cs,            		// CS pin
			.queue_size = 5, // We want to be able to queue 7 transactions at a time
			.pre_cb = spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
			.post_cb = spi_ready,			// Specify post-transfer callback
			.flags = SPI_DEVICE_HALFDUPLEX				// Halfduplex mode
		};

	esp_err_t error;

	if (spi_handle[spi_cfg->spi_host])
	{
		st7789_spi_handle = spi_handle[spi_cfg->spi_host];
	}
	else
	{
		//Initialize the SPI bus
		ESP_LOGD(TAG, "Initialization SPI%d", spi_cfg->spi_host + 1);
		error = spi_bus_initialize(spi_cfg->spi_host, &buscfg, spi_cfg->dma_channel);
		CHECK((error == ESP_OK), NULL, "SPI device %d initialize fail", spi_cfg->spi_host);
	}

	error = spi_bus_add_device(spi_cfg->spi_host, &devcfg, &st7789_spi_handle);
	CHECK((error == ESP_OK), NULL, "SPI device %d add fail", spi_cfg->spi_host);

	st7789_dev->spi_config = *spi_cfg;
	st7789_dev->spi_handle = st7789_spi_handle;
	spi_handle[spi_cfg->spi_host] = st7789_spi_handle;

	lcd_init_cmd_t *lcd_init_cmds = NULL;
	lcd_init_cmds = st7789_init_cmds;
	CHECK((lcd_init_cmds != NULL), NULL, "LCD init failed");
	ESP_LOGI(TAG, "Initialization sequence.");

	// Send all the commands
	int cmd = 0;
	while (lcd_init_cmds[cmd].databytes != 0xff)
	{
		st7789_send_cmd(lcd_init_cmds[cmd].cmd);
		st7789_send_data(lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes & 0x1F);
		if (lcd_init_cmds[cmd].databytes & 0x80)
		{
			vTaskDelay(pdMS_TO_TICKS(100));
		}
		cmd++;
	}

	return st7789_dev;
}

void st7789_enable_backlight(bool backlight)
{
#if ST7789_ENABLE_BACKLIGHT_CONTROL
    printf("%s backlight.\n", backlight ? "Enabling" : "Disabling");
    uint32_t tmp = 0;

#if (ST7789_BCKL_ACTIVE_LVL==1)
    tmp = backlight ? 1 : 0;
#else
    tmp = backlight ? 0 : 1;
#endif

    gpio_set_level(ST7789_BCKL, tmp);
#endif
}

void st7789_send_cmd(uint8_t cmd)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	xSemaphoreTake(st7789_trans_in_progress, portMAX_DELAY);

	spi_transaction_t t =
		{
			.length = 8, 			// Len is in bytes, transaction length is in bits
			.tx_buffer = &cmd,      // Data
			.user = (void*) 0, 		// D/C needs to be set to 0
		};
	spi_color_sent = false; //Mark the "lv_flush_ready" NOT needs to be called in "spi_ready"
	ESP_ERROR_CHECK(spi_device_queue_trans(st7789_spi_handle, &t, portMAX_DELAY));
}

void st7789_send_data(void *data, uint16_t length)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	if (length == 0)
		return;

	xSemaphoreTake(st7789_trans_in_progress, portMAX_DELAY);

	spi_transaction_t t =
		{
			.length = length * 8, // Len is in bytes, transaction length is in bits
			.tx_buffer = data,              // Data
			.user = (void*) 1, 		// D/C needs to be set to 1
		};
	spi_color_sent = false; //Mark the "lv_flush_ready" NOT needs to be called in "spi_ready"
	ESP_ERROR_CHECK(spi_device_queue_trans(st7789_spi_handle, &t, portMAX_DELAY));
}

void st7789_send_color(void *data, uint16_t length)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	if (length == 0)
		return;

	xSemaphoreTake(st7789_trans_in_progress, portMAX_DELAY);

	spi_transaction_t t =
		{
			.length = length * 8, // Len is in bytes, transaction length is in bits
			.tx_buffer = data,              // Data
			.user = (void*) 1, 		// D/C needs to be set to 1
		};
	spi_color_sent = true; //Mark the "lv_flush_ready" NOT needs to be called in "spi_ready"
	ESP_ERROR_CHECK(spi_device_queue_trans(st7789_spi_handle, &t, portMAX_DELAY));

}

void st7789_ready_register_event_cb(st7789_ready_cb_t cb)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	if (cb == NULL)
	{
		ESP_LOGW(TAG, "Callback function not exist");
		return;
	}
	st7789_ready_cb = cb;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
/* This function is called (in irq context!) just before a transmission starts.
 It will set the D/C line to the value indicated in the user field */
static void IRAM_ATTR spi_pre_transfer_callback(spi_transaction_t *t)
{
	ESP_EARLY_LOGD(TAG, "%s", __FUNCTION__);
	int dc = (int) t->user;
	gpio_set_level(ST7789_DC, dc);
}

static void IRAM_ATTR spi_ready(spi_transaction_t *trans)
{
	ESP_EARLY_LOGD(TAG, "%s", __FUNCTION__);
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(st7789_trans_in_progress, &pxHigherPriorityTaskWoken);

	if (!spi_color_sent)
		return;

	if (st7789_ready_cb != NULL)
		st7789_ready_cb();

	if (pxHigherPriorityTaskWoken == pdTRUE)
	{
		portYIELD_FROM_ISR();
	}
}

static void st7789_set_orientation(uint8_t orientation)
{
    // ESP_ASSERT(orientation < 4);

    const char *orientation_str[] = {
        "PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"
    };

    ESP_LOGI(TAG, "Display orientation: %s", orientation_str[orientation]);

    uint8_t data[] =
    {
#if CONFIG_LV_PREDEFINED_DISPLAY_TTGO
	0x60, 0xA0, 0x00, 0xC0
#else
	0xC0, 0x00, 0x60, 0xA0
#endif
    };

    ESP_LOGI(TAG, "0x36 command value: 0x%02X", data[orientation]);

    st7789_send_cmd(ST7789_MADCTL);
    st7789_send_data((void *) &data[orientation], 1);
}
