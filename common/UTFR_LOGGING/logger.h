#ifndef UTFR_LOGGING_LOGGER_H
#define UTFR_LOGGING_LOGGER_H

#include <stdarg.h>

#include "UTFR_UART/uart.h"

extern UART_HandleTypeDef huart2; // Or whichever UART you use

typedef enum {
  LOG_LEVEL_ERROR,
  LOG_LEVEL_WARNING,
  LOG_LEVEL_INFO,
  LOG_LEVEL_DEBUG,
  LOG_LEVEL_TRACE
} log_level_t;

/**
 * @brief Initializes the logger (creates queue and logging task).
 */
void logger_init(uart_t *_uart);

/**
 * @brief Thread-safe logging function. Sends message to logger task.
 *
 * @param level Logging level (ERROR, INFO, etc.)
 * @param fmt   printf-style format string
 * @param ...   Arguments for formatting (%d, %f)
 */
void log_message(log_level_t level, const char *formatString, ...);

/**
 * @brief Thread-safe logging function through ISR
 *
 * @param level Logging level (ERROR, INFO, etc.)
 * @param fmt   printf-style format string
 * @param ...   Arguments for formatting (%d, %f)
 */
void log_message_from_isr(log_level_t level, const char *formatString, ...);

// Convenience Macros
#define LOGE(format, ...) log_message(LOG_LEVEL_ERROR, format, ##__VA_ARGS__)
#define LOGW(format, ...) log_message(LOG_LEVEL_WARNING, format, ##__VA_ARGS__)
#define LOGI(format, ...) log_message(LOG_LEVEL_INFO, format, ##__VA_ARGS__)
#define LOGD(format, ...) log_message(LOG_LEVEL_DEBUG, format, ##__VA_ARGS__)
#define LOGT(format, ...) log_message(LOG_LEVEL_TRACE, format, ##__VA_ARGS__)

#define PREPROC_CONCAT_INNER(a, b) a##b
#define PREPROC_CONCAT(a, b)       PREPROC_CONCAT_INNER(a, b)

#define LOG_ONCE_IMPL(id, level, format, ...)                                                      \
  do {                                                                                             \
    static int PREPROC_CONCAT(logged_, id) = 0;                                                    \
    if (!PREPROC_CONCAT(logged_, id)) {                                                            \
      PREPROC_CONCAT(logged_, id) = 1;                                                             \
      log_message(level, format, ##__VA_ARGS__);                                                   \
    }                                                                                              \
  } while (0)

#define LOGE_ONCE(format, ...) LOG_ONCE_IMPL(__COUNTER__, LOG_LEVEL_ERROR, format, ##__VA_ARGS__)
#define LOGW_ONCE(format, ...) LOG_ONCE_IMPL(__COUNTER__, LOG_LEVEL_WARNING, format, ##__VA_ARGS__)
#define LOGI_ONCE(format, ...) LOG_ONCE_IMPL(__COUNTER__, LOG_LEVEL_INFO, format, ##__VA_ARGS__)
#define LOGD_ONCE(format, ...) LOG_ONCE_IMPL(__COUNTER__, LOG_LEVEL_DEBUG, format, ##__VA_ARGS__)
#define LOGT_ONCE(format, ...) LOG_ONCE_IMPL(__COUNTER__, LOG_LEVEL_TRACE, format, ##__VA_ARGS__)

#endif
