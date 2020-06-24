/**
 * @file lcd-driver.c
 * @brief Driver for the LCD panel
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "rom/uart.h"
#include "smbus.h"
#include "i2c-lcd1602.h"

#undef USE_STDIN

#define DELAY		             500

char *menuItems;

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

void scroll_up(char *A[], int size, struct i2c_lcd1602_info_t *lcd_info)
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

void init_menu_list(char* list, struct i2c_lcd1602_info_t *lcd_info) {
    menuItems = list;
    _wait_for_user();

    for (int i = 0; i < 4; ++i)
    {
        i2c_lcd1602_move_cursor(lcd_info, 0, i);
        i2c_lcd1602_write_string(lcd_info, menuItems[i]);
    }
}


