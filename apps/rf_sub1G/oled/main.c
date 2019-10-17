/****************************************************************************
 *   apps/rf_sub1G/oled/main.c
 *
 * sub1G_module support code - USB version
 *
 * Copyright 2013-2014 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *************************************************************************** */


#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/ssp.h"
#include "drivers/adc.h"
#include "drivers/i2c.h"

#include "extdrv/status_led.h"
#include "extdrv/tmp101_temp_sensor.h"
#include "extdrv/ssd130x_oled_driver.h"
#include "extdrv/ssd130x_oled_buffer.h"
#include "lib/font.h"
#include "extdrv/bme280_humidity_sensor.h"
#include "extdrv/veml6070_uv_sensor.h"
#include "extdrv/tsl256x_light_sensor.h"

#define MODULE_VERSION	0x03
#define MODULE_NAME "RF Sub1G - USB"


#define DEBUG 1
#define BUFF_LEN 60

#define SELECTED_FREQ  FREQ_SEL_48MHz

static volatile uint32_t update_display = 0;

/***************************************************************************** */
/* Pins configuration */
/* pins blocks are passed to set_pins() for pins configuration.
 * Unused pin blocks can be removed safely with the corresponding set_pins() call
 * All pins blocks may be safelly merged in a single block for single set_pins() call..
 */
const struct pio_config common_pins[] = {
	/* UART 0 */
	{ LPC_UART0_RX_PIO_0_1,  LPC_IO_DIGITAL },
	{ LPC_UART0_TX_PIO_0_2,  LPC_IO_DIGITAL },
	/* I2C 0 */
	{ LPC_I2C0_SCL_PIO_0_10, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	{ LPC_I2C0_SDA_PIO_0_11, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	/* RESET PIN */
	{ LPC_GPIO_0_0, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },
	ARRAY_LAST_PIO,
};


const struct pio temp_alert = LPC_GPIO_0_3;
const struct pio status_led_green = LPC_GPIO_0_28;
const struct pio status_led_red = LPC_GPIO_0_29;



/***************************************************************************** */
void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	gpio_on();
	/* System tick timer MUST be configured and running in order to use the sleeping
	 * functions */
	systick_timer_on(1); /* 1ms */
	systick_start();
}

/* Define our fault handler. This one is not mandatory, the dummy fault handler
 * will be used when it's not overridden here.
 * Note : The default one does a simple infinite loop. If the watchdog is deactivated
 * the system will hang.
 */
void fault_info(const char* name, uint32_t len)
{
	uprintf(UART0, name);
	while (1);
}



/***************************************************************************** */
/* Temperature */

#define TMP101_ADDR  0x94  /* Pin Addr0 (pin5 of tmp101) connected to VCC */
struct tmp101_sensor_config tmp101_sensor = {
	.bus_num = I2C0,
	.addr = TMP101_ADDR,
	.resolution = TMP_RES_ELEVEN_BITS,
};

/* The Temperature Alert pin is on GPIO Port 0, pin 7 (PIO0_7) */
/* The I2C Temperature sensor is at address 0x94 */
void WAKEUP_Handler(void)
{
}

void temp_config(int uart_num)
{
	int ret = 0;

	/* Temp Alert */
	config_gpio(&temp_alert, LPC_IO_MODE_PULL_UP, GPIO_DIR_IN, 0);
	/* FIXME : add a callback on temp_alert edge */

	/* Temp sensor */
	ret = tmp101_sensor_config(&tmp101_sensor);
	if (ret != 0) {
		uprintf(uart_num, "Temp config error: %d\n\r", ret);
	} else {
		uprintf(uart_num, "Temp config done.\n\r");
	}

}

void temp_display(int uart_num, int* deci_degrees)
{
	uint16_t val = 0;
	int ret = 0;
	/* Read the temperature */
	ret = tmp101_sensor_read(&tmp101_sensor, &val, deci_degrees);
	if (ret != 0) {
		uprintf(uart_num, "Temp read error: %d\n\r", ret);
	} else {
		uprintf(uart_num, "Temp read: %d,%d - raw: 0x%04x.\n\r",
				(*deci_degrees/10), (*deci_degrees%10), val);
	}
}

/***************************************************************************** */
/* Luminosity */

/* Note : These are 8bits address */
#define TSL256x_ADDR   0x52 /* Pin Addr Sel (pin2 of tsl256x) connected to GND */
struct tsl256x_sensor_config tsl256x_sensor = {
	.bus_num = I2C0,
	.addr = TSL256x_ADDR,
	.gain = TSL256x_LOW_GAIN,
	.integration_time = TSL256x_INTEGRATION_100ms,
	.package = TSL256x_PACKAGE_T,
};

void lux_config(int uart_num)
{
	int ret = 0;
	ret = tsl256x_configure(&tsl256x_sensor);
	if (ret != 0) {
		uprintf(uart_num, "Lux config error: %d\n\r", ret);
	}
}

void lux_display(int uart_num, uint16_t* ir, uint32_t* lux)
{
	uint16_t comb = 0;
	int ret = 0;

	ret = tsl256x_sensor_read(&tsl256x_sensor, &comb, ir, lux);
	if (ret != 0) {
		uprintf(uart_num, "Lux read error: %d\n\r", ret);
	} else {
		uprintf(uart_num, "Lux: %d  (Comb: 0x%04x, IR: 0x%04x)\n\r", *lux, comb, *ir);
	}
}

/***************************************************************************** */
/* BME280 Sensor */

/* Note : 8bits address */
#define BME280_ADDR   0xEC
struct bme280_sensor_config bme280_sensor = {
	.bus_num = I2C0,
	.addr = BME280_ADDR,
	.humidity_oversampling = BME280_OS_x16,
	.temp_oversampling = BME280_OS_x16,
	.pressure_oversampling = BME280_OS_x16,
	.mode = BME280_NORMAL,
	.standby_len = BME280_SB_62ms,
	.filter_coeff = BME280_FILT_OFF,
};

void bme_config(int uart_num)
{
	int ret = 0;

	ret = bme280_configure(&bme280_sensor);
	if (ret != 0) {
		uprintf(uart_num, "Sensor config error: %d\n\r", ret);
	}
}

/* BME will obtain temperature, pressure and humidity values */

void bme_display(int uart_num, uint32_t* pressure, uint32_t* temp, uint16_t* humidity)
{
	int ret = 0;

	ret = bme280_sensor_read(&bme280_sensor, pressure, temp, humidity);
	if (ret != 0) {
		uprintf(uart_num, "Sensor read error: %d\n\r", ret);
	} else {
		int comp_temp = 0;
		uint32_t comp_pressure = 0;
		uint32_t comp_humidity = 0;

		comp_temp = bme280_compensate_temperature(&bme280_sensor, *temp) / 10;
		comp_pressure = bme280_compensate_pressure(&bme280_sensor, *pressure) / 100;
		comp_humidity = bme280_compensate_humidity(&bme280_sensor, *humidity) / 10;
		uprintf(uart_num, "P: %d hPa, T: %d,%02d degC, H: %d,%d rH\n\r",
				comp_pressure,
				comp_temp / 10,  (comp_temp > 0) ? (comp_temp % 10) : ((-comp_temp) % 10),
				comp_humidity / 10, comp_humidity % 10);
		*temp = comp_temp;
		*pressure = comp_pressure;
		*humidity = comp_humidity;
	}
}


/***************************************************************************** */
/* Adafruit Oled Display */
/* #define DISPLAY_ADDR   0x7A */
/* For other OLED Displays maybe : 0x78*/
#define DISPLAY_ADDR   0x7A
static uint8_t gddram[ 4 + GDDRAM_SIZE ];
struct oled_display display = {
	.bus_type = SSD130x_BUS_I2C,
	.address = DISPLAY_ADDR,
	.bus_num = I2C0,
	.charge_pump = SSD130x_INTERNAL_PUMP,
	.gpio_rst = LPC_GPIO_0_0,
	.video_mode = SSD130x_DISP_NORMAL,
	.contrast = 128,
	.scan_dir = SSD130x_SCAN_BOTTOM_TOP,
	.read_dir = SSD130x_RIGHT_TO_LEFT,
	.display_offset_dir = SSD130x_MOVE_TOP,
	.display_offset = 4,
  .gddram = gddram,
};

#define ROW(x)   VERTICAL_REV(x)
DECLARE_FONT(font);

void display_char(uint8_t line, uint8_t col, uint8_t c)
{
	uint8_t tile = (c > FIRST_FONT_CHAR) ? (c - FIRST_FONT_CHAR) : 0;
	uint8_t* tile_data = (uint8_t*)(&font[tile]);
	ssd130x_buffer_set_tile(gddram, col, line, tile_data);
}

int display_line(uint8_t line, uint8_t col, char* text)
{
	int len = strlen((char*)text);
	int i = 0;

	for (i = 0; i < len; i++) {
		uint8_t tile = (text[i] > FIRST_FONT_CHAR) ? (text[i] - FIRST_FONT_CHAR) : 0;
		uint8_t* tile_data = (uint8_t*)(&font[tile]);
		ssd130x_buffer_set_tile(gddram, col++, line, tile_data);
		if (col >= (SSD130x_NB_COL / 8)) {
			col = 0;
			line++;
			if (line >= SSD130x_NB_PAGES) {
				return i;
			}
		}
	}
	return len;
}



/***************************************************************************** */
void periodic_display(uint32_t tick)
{
	update_display = 1;
}

int main(void)
{
	int ret = 0;
	system_init();
	uart_on(UART0, 115200, NULL);
	i2c_on(I2C0, I2C_CLK_100KHz, I2C_MASTER);
	status_led_config(&status_led_green, &status_led_red);

	/* Temperature sensor */
	temp_config(UART0);
	lux_config(UART0);
	bme_config(UART0);

	/* Configure and start display */
	ret = ssd130x_display_on(&display);
	/* Erase screen with lines, makes it easier to know things are going right */
	ssd130x_buffer_set(gddram, 0x00);
	ret = ssd130x_display_full_screen(&display);
	

	/* Add periodic handler */
	add_systick_callback(periodic_display, 1000);

	uprintf(UART0, "App started\n\r");

	while (1) {
		
		/* Request a Temp conversion on I2C TMP101 temperature sensor */
		tmp101_sensor_start_conversion(&tmp101_sensor); /* A conversion takes about 40ms */
		
		/* Tell we are alive :) */
		chenillard(250);

		/* Display */
		if (update_display == 1) {
			int deci_degrees = 0;
			char data[20];
			uint16_t ir = 0, humidity = 0;
			uint32_t pressure = 0, temp = 0, lux = 0;

			uprintf(UART0, "Updating display\n\r");

			/* Read the temperature sensor on board */
			temp_display(UART0, &deci_degrees);
			lux_display(UART0, &ir, &lux);
			bme_display(UART0, &pressure, &temp, &humidity);

			/* Update display */
			snprintf(data, 20, "Temp: %d,%d", (temp / 10), (temp % 10));
			display_line(2, 0, data);
			snprintf(data, 20, "Lum: %d", lux);
			display_line(3, 0, data);
			snprintf(data, 20, "Hum: %d,%d", humidity/10, humidity%10);
			display_line(4, 0, data);
			/* And send to screen */
			ret = ssd130x_display_full_screen(&display);
			if (ret < 0) {
				uprintf(UART0, "Display update error: %d\n\r", ret);
			}
			update_display = 0;
		}
	}
	return 0;
}
