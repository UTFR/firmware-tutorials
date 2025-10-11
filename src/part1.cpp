#include <Arduino.h>
#include <FreeRTOS.h>

#define LED_PIN 

void blinkTask(void *pvParameters) {
  pinMode(LED_PIN, OUTPUT);
  while (1) {
    if (xQueueReceive(countQueue, &count, portMAX_DELAY) == pdPASS) {
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(300));
      digitalWrite(LED_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(300));
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  xTaskCreate(blinkTask, "Blink", 256, NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop() {}
