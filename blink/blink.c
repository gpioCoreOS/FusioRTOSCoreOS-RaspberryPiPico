#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/i2c.h"   // Notwendig für I2C-Initialisierung
#include "hardware/gpio.h"  // Notwendig für GPIO-Funktionen

// Include your LCD driver header
// Since lcd_1602_i2c.h is now in the same directory as blink.c,
// a simple include is sufficient.
#include "lcd_1602_i2c.h"

// --- LED Task Configuration ---
#define LED_PIN PICO_DEFAULT_LED_PIN
#define BLINK_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define BLINK_DELAY_MS 500

// --- LCD Task Configuration ---
#define LCD_TASK_PRIORITY (tskIDLE_PRIORITY + 2) // Höhere Priorität als LED-Task
#define LCD_I2C_INSTANCE    i2c0    // Welchen I2C-Hardware-Block nutzen wir?
#define LCD_I2C_SDA_PIN     4       // GPIO-Pin für I2C SDA (z.B. GP4)
#define LCD_I2C_SCL_PIN     5       // GPIO-Pin für I2C SCL (z.B. GP5)
#define LCD_I2C_ADDR        0x27    // Standard I2C-Adresse für PCF8574-basierte LCDs (kann 0x3F sein)

// Global instance of the LCD structure
// This instance will be passed to the LCD driver functions.
lcd_1602_i2c_t my_lcd;

// --- Task function to blink the LED ---
void vBlinkTask(void *pvParameters) {
    // Initialize the GPIO pin as an output
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    for (;;) {
        // Turn the LED on
        gpio_put(LED_PIN, 1);
        printf("LED an!\n");
        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY_MS)); // Wait 500ms

        // Turn the LED off
        gpio_put(LED_PIN, 0);
        printf("LED aus!\n");
        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY_MS)); // Wait 500ms
    }
}

// --- Task function to control the LCD ---
void vLcdTask(void *pvParameters) {
    // 1. Initialize I2C hardware
    // Default clock is 100kHz, many LCDs also work with 400kHz.
    i2c_init(LCD_I2C_INSTANCE, 100 * 1000); // 100 kHz I2C clock

    // Set GPIOs to I2C function
    gpio_set_function(LCD_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(LCD_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(LCD_I2C_SDA_PIN); // Enable pull-ups, as most boards don't have external ones
    gpio_pull_up(LCD_I2C_SCL_PIN);

    // 2. Initialize LCD driver
    // Pass the address of the global LCD structure
    lcd_init(&my_lcd, LCD_I2C_INSTANCE, LCD_I2C_ADDR);
    printf("LCD initialization completed in task.\n");

    // 3. Write content to LCD
    lcd_clear(&my_lcd);
    lcd_set_cursor(&my_lcd, 0, 0); // Row 0, Column 0
    lcd_write_string(&my_lcd, "Hallo FreeRTOS!");
    lcd_set_cursor(&my_lcd, 1, 0); // Row 1, Column 0
    lcd_write_string(&my_lcd, "Pico LCD I2C");

    for (;;) {
        // Example: Blink a dot on the LCD
        static int dot_state = 0;
        lcd_set_cursor(&my_lcd, 1, 15); // Last column of the second row
        if (dot_state == 0) {
            lcd_write_char(&my_lcd, '.');
            dot_state = 1;
        } else {
            lcd_write_char(&my_lcd, ' ');
            dot_state = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Wait 500ms
    }
}

// --- Main function of the program ---
int main() {
    // Initialize standard Pico libraries
    // This also includes the USB serial interface for printf
    stdio_init_all();

    printf("FreeRTOS LED and LCD Example on Raspberry Pi Pico\n");

    // Create the LED blink task
    // xTaskCreate(Task-Function, Task-Name, Stack-Size, Parameters, Priority, Task-Handle)
    if (xTaskCreate(vBlinkTask, "BlinkTask", configMINIMAL_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL) != pdPASS) {
        printf("Error: BlinkTask could not be created!\n");
        while (1) {
            // Endless loop on error
        }
    }

    // Create the LCD task
    // The stack is slightly larger, as string operations and I2C might require more.
    if (xTaskCreate(vLcdTask, "LcdTask", configMINIMAL_STACK_SIZE * 2, NULL, LCD_TASK_PRIORITY, NULL) != pdPASS) {
        printf("Error: LcdTask could not be created!\n");
        while (1) {
            // Endless loop on error
        }
    }

    // Start the FreeRTOS scheduler
    // This passes control to FreeRTOS, which then manages the tasks.
    vTaskStartScheduler();

    // The code should never reach this point unless there is a critical error in the scheduler.
    printf("Error: Scheduler terminated!\n");
    for (;;) {
        // Endless loop on scheduler termination
    }

    return 0;
}
