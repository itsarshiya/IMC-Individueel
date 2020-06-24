/*
 * MIT License
 *
 * Copyright (c) 2018 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file app_main.c
 * @brief Example application for the LCD1602 16x2 Character Dot Matrix LCD display via I2C backpack..
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "rom/uart.h"

#include "smbus.h"
#include "i2c-lcd1602.h"

#define TAG "app"

// Undefine USE_STDIN if no stdin is available (e.g. no USB UART) - a fixed delay will occur instead of a wait for a keypress.
//#define USE_STDIN  1
#undef USE_STDIN

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL
#define LCD_NUM_ROWS			 4
#define LCD_NUM_COLUMNS			 40
#define LCD_NUM_VIS_COLUMNS		 20

#define DELAY		             500

char *menuItems;

static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}

static uint8_t _wait_for_user(void)
{
    uint8_t c = 0;
#ifdef USE_STDIN
    while (!c)
    {
       STATUS s = uart_rx_one_char(&c);
       if (s == OK) {
          printf("%c", c);
       }
    }
#else
    vTaskDelay(1000 / portTICK_RATE_MS);
#endif
    return c;
}

void init_menu_list(char* list, struct i2c_lcd1602_info_t *lcd_info) {
    menuItems = list;
    _wait_for_user();

    for (int i = 0; i < 3; ++i)
    {
        i2c_lcd1602_move_cursor(lcd_info, 0, i);
        i2c_lcd1602_write_string(lcd_info, menuItems[i]);
    }
}

void init_bars(i2c_lcd1602_info_t * lcd_info) {
    uint8_t bar1[8]  = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1f};
    uint8_t bar2[8]  = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1f, 0x1f};
    uint8_t bar3[8]  = {0x0, 0x0, 0x0, 0x0, 0x0, 0x1f, 0x1f, 0x1f};
    uint8_t bar4[8]  = {0x0, 0x0, 0x0, 0x0, 0x1f, 0x1f, 0x1f, 0x1f};
    uint8_t bar5[8]  = {0x0, 0x0, 0x0, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};
    uint8_t bar6[8]  = {0x0, 0x0, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};
    uint8_t bar7[8]  = {0x0, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};
    uint8_t bar8[8]  = {0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};

    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_0, bar1);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_1, bar2);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_2, bar3);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_3, bar4);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_4, bar5);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_5, bar6);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_6, bar7);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_7, bar8);
}

void scroll_down(char *A[], struct i2c_lcd1602_info_t *lcd_info)
{
    int size = sizeof(A)/sizeof(A[0]);

	char *last = A[size - 1];
	for (int i = size - 2; i >= 0; i--)
		A[i + 1] = A[i];
        
	A[0] = last;

    vTaskDelay(DELAY / portTICK_RATE_MS);
    i2c_lcd1602_clear(lcd_info);
    vTaskDelay(DELAY / portTICK_RATE_MS);
}

void scroll_up(char *A[], struct i2c_lcd1602_info_t *lcd_info)
{
    int size = sizeof(A)/sizeof(A[0]);

	char *first = A[0];
	for (int i = 0; i < size - 1; i++)
		A[i] = A[i + 1];

	A[size - 1] = first;

    vTaskDelay(DELAY / portTICK_RATE_MS);
    i2c_lcd1602_clear(lcd_info);
    vTaskDelay(DELAY / portTICK_RATE_MS);
}


void lcd1602_task(void * pvParameter)
{
    // Set up I2C
    i2c_master_init();
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = CONFIG_LCD1602_I2C_ADDRESS;

    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    smbus_init(smbus_info, i2c_num, address);
    smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS);

    // Set up the LCD1602 device with backlight off
    i2c_lcd1602_info_t * lcd_info = i2c_lcd1602_malloc();
    i2c_lcd1602_init(lcd_info, smbus_info, true, LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VIS_COLUMNS);
    
    char *menuItems[7];
    menuItems[0] = "Music";
    menuItems[1] = "Settings";
    menuItems[2] = "Equalizer";
    menuItems[3] = "Radio";
    menuItems[4] = "Photos";
    menuItems[5] = "Driver";
    menuItems[6] = "Stop";
    menuItems[7] = "Start";

    int menuItemsSize = sizeof(menuItems)/sizeof(menuItems[0]);

    for (int i = 0; i < 3; ++i)
    {
        i2c_lcd1602_move_cursor(lcd_info, 0, i);
        i2c_lcd1602_write_string(lcd_info, menuItems[i]);
    }
   
    init_bars(lcd_info);

    _wait_for_user();

    char bars[8] = {I2C_LCD1602_CHARACTER_CUSTOM_0,
                    I2C_LCD1602_CHARACTER_CUSTOM_1,
                    I2C_LCD1602_CHARACTER_CUSTOM_2,
                    I2C_LCD1602_CHARACTER_CUSTOM_3,
                    I2C_LCD1602_CHARACTER_CUSTOM_4,
                    I2C_LCD1602_CHARACTER_CUSTOM_5,
                    I2C_LCD1602_CHARACTER_CUSTOM_6,
                    I2C_LCD1602_CHARACTER_CUSTOM_7,
                    I2C_LCD1602_CHARACTER_CUSTOM_1};

    while (1)
    {
        char spectrum[8]= {""};
        
        for (int i = 0; i < 9; ++i)
        {
            int random = rand()%8;
            spectrum[i] = bars[random];
        }

        i2c_lcd1602_move_cursor(lcd_info, 0, 5);
        i2c_lcd1602_write_string(lcd_info, spectrum);
        vTaskDelay(150 / portTICK_RATE_MS);
    }

    while (1) {
        scroll_down(menuItems,lcd_info);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}


void app_main()
{
    xTaskCreate(&lcd1602_task, "lcd1602_task", 4096, NULL, 5, NULL);
}

