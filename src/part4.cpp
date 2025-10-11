#include <Arduino.h>
#include <FreeRTOS.h>
#include <semphr.h>

#define LED_PIN 

SemaphoreHandle_t blinkSemaphore;

void blinkTask(void *pvParameters) {
  pinMode(LED_PIN, OUTPUT);
  while (1) {
    // Wait until semaphore is given
    if (xSemaphoreTake(blinkSemaphore, portMAX_DELAY) == pdTRUE) {
      // blicnk once 
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(300));
      digitalWrite(LED_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(300));
    }
  }
}

void counterTask(void *pvParameters) {
  int count = 0;
  while (1) {
    count++;
    Serial.printf("Count: %d\n", count);
    vTaskDelay(pdMS_TO_TICKS(500));  // Delay 1s between prints

    // for every 5 counts, release semaphore to signal LED task
    if (count % 5 == 0) {
      Serial.println("mult of 5 reached. giving semaphore...");
      xSemaphoreGive(blinkSemaphore);
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) ; 

  blinkSemaphore = xSemaphoreCreateBinary();

  if (blinkSemaphore != NULL) {
    xTaskCreate(blinkTask, "Blink", 256, NULL, 1, NULL);
    xTaskCreate(counterTask, "Counter", 256, NULL, 1, NULL);
    vTaskStartScheduler();
  } else {
    Serial.println("Failed to create semaphore!");
  }
}

void loop() {}
