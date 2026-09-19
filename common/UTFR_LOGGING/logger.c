/*
Logic for Thread-Safe Logger Implementation Using FreeRTOS Queue and UART Output
*/

#include "logger.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "UTFR_UART/uart.h"
#include "queue.h"
#include "utfr_hal.h"

// Configuring the logger queue
#define LOGGER_QUEUE_LENGTH       128
#define LOGGER_MSG_MAX_LEN        128
#define LOGGER_MSG_BUFFER_PADDING 16
#define LOGGER_TASK_STACK_SIZE    512
#define LOGGER_TASK_PRIORITY      (tskIDLE_PRIORITY + 1) // Probably a low priority task

// Struct for log message itself
typedef struct {
  log_level_t level;
  uint32_t timestamp;
  char msg[LOGGER_MSG_MAX_LEN];
} logger_msg_t;

// Static variables
static QueueHandle_t logger_queue = NULL;      // FreeRTOS queue struct
static TaskHandle_t logger_task_handle = NULL; // Stores handle to logger task

// Internal function declarations
static void logger_task(void *params);
static void logger_output(const char *msg, int len);

static uart_t *uart = NULL;

// Initializes the logger
void logger_init(uart_t *_uart) {
  uart = _uart;
  if (logger_queue == NULL) {
    logger_queue = xQueueCreate(LOGGER_QUEUE_LENGTH, sizeof(logger_msg_t));
    if (logger_queue != NULL) {
      xTaskCreate(logger_task, "LoggerTask", LOGGER_TASK_STACK_SIZE, NULL, LOGGER_TASK_PRIORITY,
                  &logger_task_handle);
    }
  }
}

// Logger API
void log_message(log_level_t level, const char *formatString, ...) {
  if (logger_queue == NULL) { return; }
  logger_msg_t log_msg;
  log_msg.level = level;                   // DEBUG, ERROR...
  log_msg.timestamp = xTaskGetTickCount(); // use current tick count as timestamp

  // Variadic argument handling
  va_list args;
  va_start(args, formatString);
  vsnprintf(log_msg.msg, LOGGER_MSG_MAX_LEN, formatString,
            args);                       // Limits the number of chars written to the buffer
  va_end(args);
  xQueueSend(logger_queue, &log_msg, 0); // Send the log message
}

// Log function from an ISR
void log_message_from_isr(log_level_t level, const char *formatString, ...) {
  if (logger_queue == NULL) { return; }
  logger_msg_t log_msg;
  log_msg.level = level;

  // Varadic argument handling
  va_list args;
  va_start(args, formatString);
  vsnprintf(log_msg.msg, LOGGER_MSG_MAX_LEN, formatString, args);
  va_end(args);

  BaseType_t xHigherPriorityTaskWoken = pdFALSE; // No higher priority task has been unblocked yet
  xQueueSendFromISR(logger_queue, &log_msg, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(
    xHigherPriorityTaskWoken); // Context switch after ISR if woke up higher priority task
}

// Logger task
static void logger_task(void *params) {
  (void)params;
  logger_msg_t log_msg;

  for (;;) {
    xQueueReceive(logger_queue, &log_msg, portMAX_DELAY);
    char outputBuffer[LOGGER_MSG_MAX_LEN + LOGGER_MSG_BUFFER_PADDING];
    const char *levelStr = "UNKNOWN";
    switch (log_msg.level) {
    case LOG_LEVEL_ERROR:   levelStr = "ERROR"; break;
    case LOG_LEVEL_DEBUG:   levelStr = "DEBUG"; break;
    case LOG_LEVEL_INFO:    levelStr = "INFO"; break;
    case LOG_LEVEL_TRACE:   levelStr = "TRACE"; break;
    case LOG_LEVEL_WARNING: levelStr = "WARNING"; break;
    }
    uint32_t timestamp_ms = log_msg.timestamp * portTICK_PERIOD_MS;
    int len = snprintf(outputBuffer, sizeof(outputBuffer), "[%08lu] [%s] %s\r\n", timestamp_ms,
                       levelStr, log_msg.msg);
    logger_output(outputBuffer, len);
  }
}

// Outputs to serial monitor via UART (send back to queue if not successful)
static void logger_output(const char *msg, int len) { printf("%.*s", len, msg); }
