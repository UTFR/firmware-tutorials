#include <Arduino.h>
#include <FreeRTOS.h>

#define LED_PIN 
QueueHandle_t countQueue;

void blinkTask(void *pvParameters) {
  pinMode(LED_PIN, OUTPUT);
  int count = 0;
  while (1) {
    if (xQueueReceive(countQueue, &count, portMAX_DELAY) == pdPASS) {
      for (int i = 0; i < count; i++) {
        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(300));
        digitalWrite(LED_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(300));
      }
      vTaskDelay(pdMS_TO_TICKS(2000));  // Pause after series
    }
  }
}

void counterTask(void *pvParameters) {
  int count = 1;
  while (1) {
    xQueueSend(countQueue, &count, 0);
    count++;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  countQueue = xQueueCreate(5, sizeof(int));

  if (countQueue != NULL) {
    xTaskCreate(blinkTask, "Blink", 256, NULL, 1, NULL);
    xTaskCreate(counterTask, "Counter", 256, NULL, 1, NULL);
    vTaskStartScheduler();
  } else {
    Serial.println("Failed to create queue!");
  }
}

void loop() {}
