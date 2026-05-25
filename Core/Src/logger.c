#include "logger.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_uart.h"
#include "usart.h"
#include <inttypes.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// The number of logs the logger can store between his scheduling frames.
#define RING_BUFFER_SIZE 32
#define MAX_LOGGING_MESSAGE_LENGTH 300

static const char *RING_BUFFER_OVERFLOW_MESSAGE =
    "This log call caused the ring buffer to overflow.";

static char ring_buffer[RING_BUFFER_SIZE][MAX_LOGGING_MESSAGE_LENGTH];
static uint16_t write_index = 0;
static uint16_t read_index = 0;

static log_level_t level = LOG_WARNING;
static log_format_t format = PLAIN_TEXT;
static log_destination_t destination = UART;

const uint16_t LOGGER_RUN_PERIOD = 100;

/**
 * Get the index of the current head of the buffer.
 * @return - The index of the element or [-1] if the buffer is empty
 */
static int ring_buffer_get_index() {
  if (read_index == write_index) {
    return -1;
  }

  uint16_t tmp = read_index;
  read_index = (read_index + 1) % RING_BUFFER_SIZE;
  return tmp;
}

/**
 * To string method for the log level
 */
static char *log_level_to_string(log_level_t level) {
  switch (level) {
  case LOG_DEBUG:
    return "DEBUG";
  case LOG_INFO:
    return "INFO";
  case LOG_WARNING:
    return "WARNING";
  case LOG_ERROR:
    return "ERROR";
  case LOG_CRITICAL:
    return "CRITICAL";
  default:
    return "UNKNOWN";
  }
}

/**
 * To string method for a given log record.
 * Respects the log format
 */
static int log_to_string(log_level_t level, const char *filename,
                         uint16_t line_number, uint32_t time,
                         const char *message, char *buf, size_t buf_size) {

  int len;

  switch (format) {
  case CSV:
    // Concatenate all parts of the log
    len = snprintf(buf, buf_size, "%" PRIu32 ",%s,%s,%" PRIu16 ",%s\n", time,
                   log_level_to_string(level), filename, line_number, message);
    break;
  case PLAIN_TEXT:
  default:
    // Concatenate all parts of the log
    len =
        snprintf(buf, buf_size, "[%" PRIu32 "] %s %s:%" PRIu16 " - %s\n", time,
                 log_level_to_string(level), filename, line_number, message);
    break;
  }

  return len;
}

log_level_t logger_get_log_level() { return level; }

void logger_init(log_level_t level_param, log_format_t format_param,
                 log_destination_t destination_param) {
  level = level_param;
  format = format_param;
  destination = destination_param;
}

int logger_log(log_level_t level, log_module_t module, char *filename,
               uint16_t line_number, char *message, ...) {
  if (level < logger_get_log_level()) {
    return 0;
  }
  if (!(*module.enabled)) {
    return 0;
  }

  // ring buffer is full
  if ((write_index + 1) % RING_BUFFER_SIZE == read_index) {
    // Overwrite the oldest entry with a warning
    log_to_string(LOG_WARNING, filename, line_number, HAL_GetTick(),
                  RING_BUFFER_OVERFLOW_MESSAGE, ring_buffer[read_index],
                  MAX_LOGGING_MESSAGE_LENGTH);

    return 0;
  }

  // Append to the ring buffer
  va_list args;
  va_start(args, message);

  char formatted_message[MAX_LOGGING_MESSAGE_LENGTH];
  vsnprintf(formatted_message, MAX_LOGGING_MESSAGE_LENGTH, message, args);

  // Write the log into the correct entry of the ring buffer
  log_to_string(level, filename, line_number, HAL_GetTick(), formatted_message,
                ring_buffer[write_index], MAX_LOGGING_MESSAGE_LENGTH);

  va_end(args);

  ring_buffer[write_index][MAX_LOGGING_MESSAGE_LENGTH - 1] = '\0';

  write_index = (write_index + 1) % RING_BUFFER_SIZE;

  return 1;
}

void logger_run() {
  char combined_buf[RING_BUFFER_SIZE * MAX_LOGGING_MESSAGE_LENGTH];
  size_t len = 0;

  while (read_index != write_index) {
    size_t msg_len = strlen(ring_buffer[read_index]);
    if (len + msg_len >= sizeof(combined_buf)) {
      break;
    }

    memcpy(combined_buf + len, ring_buffer[read_index], msg_len);
    len += msg_len;

    read_index = (read_index + 1) % RING_BUFFER_SIZE;
  }

  switch (destination) {
  case UART:
    HAL_UART_Transmit(&huart2, (uint8_t *)combined_buf, len, 1000000);
    return;
  default:
    return;
  }
}
