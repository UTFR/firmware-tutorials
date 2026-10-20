#pragma once
#include <cstdint>
using TickType_t=uint32_t;
constexpr unsigned configTICK_RATE_HZ=1000;
constexpr int pdPASS=1;
constexpr uint32_t portMAX_DELAY=UINT32_MAX;
constexpr TickType_t pdMS_TO_TICKS(uint32_t ms) { return ms; }
TickType_t xTaskGetTickCount();
void vTaskDelayUntil(TickType_t *, TickType_t);
int xTaskCreate(void (*)(void *), const char *, unsigned, void *, unsigned, void *);
void vTaskStartScheduler();
