#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// Der GPIO-Pin, an dem die LED angeschlossen ist (Onboard-LED ist GPIO 25)
#define LED_PIN PICO_DEFAULT_LED_PIN

// Die Priorität des LED-Blink-Tasks
#define BLINK_TASK_PRIORITY (tskIDLE_PRIORITY + 1)

// Die Verzögerungszeit für das Blinken (in Millisekunden)
#define BLINK_DELAY_MS 500

// Task-Funktion zum Blinken der LED
void vBlinkTask(void *pvParameters) {
    // Initialisiere den GPIO-Pin als Ausgang
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    for (;;) {
        // Schalte die LED ein
        gpio_put(LED_PIN, 1);
        printf("LED an!\n");
        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY_MS)); // Warte 500ms

        // Schalte die LED aus
        gpio_put(LED_PIN, 0);
        printf("LED aus!\n");
        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY_MS)); // Warte 500ms
    }
}

// Hauptfunktion des Programms
int main() {
    // Initialisiere die Standard-Pico-Bibliotheken
    // Dies beinhaltet auch die USB-Seriell-Schnittstelle für printf
    stdio_init_all();

    printf("FreeRTOS LED Blink Beispiel auf Raspberry Pi Pico\n");

    // Erstelle den LED-Blink-Task
    // xTaskCreate(Task-Funktion, Task-Name, Stack-Größe, Parameter, Priorität, Task-Handle)
    if (xTaskCreate(vBlinkTask, "BlinkTask", configMINIMAL_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL) != pdPASS) {
        printf("Fehler: BlinkTask konnte nicht erstellt werden!\n");
        while (1) {
            // Endlosschleife bei Fehler
        }
    }

    // Starte den FreeRTOS Scheduler
    // Dies übergibt die Kontrolle an FreeRTOS, das dann die Tasks verwaltet.
    vTaskStartScheduler();

    // Der Code sollte diesen Punkt niemals erreichen, es sei denn, es gibt einen kritischen Fehler im Scheduler.
    printf("Fehler: Scheduler wurde beendet!\n");
    for (;;) {
        // Endlosschleife bei Scheduler-Ende
    }

    return 0;
}