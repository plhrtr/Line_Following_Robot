#ifndef LOGGER_H
#define LOGGER_H

#include <stdarg.h>
#include <stdint.h>

// The pause between each logger runs
extern const uint16_t LOGGER_RUN_FREQ;

/**
 * Struct to define a module that logs messages.
 */
typedef struct {
  // The name of the module.
  char *name;
  // Pointer to a flag whether the module is enabled.
  char *enabled;
} log_module_t;

/**
 * Logging levels for the logger.
 */
typedef enum {
  // Detailed diagnostic informations.
  LOG_DEBUG,
  // General informational messages about normal operations.
  LOG_INFO,
  // Warning messages for potentially problematic situations.
  LOG_WARNING,
  // Error messages.
  LOG_ERROR,
  // Critical or fatal errors, that may cause the application to fail.
  LOG_CRITICAL,
} log_level_t;

/**
 * Record holding all the informations for a single log entry.
 */
typedef struct {
  // The level of this log message.
  log_level_t level;
  // The file the log is from.
  char *file;
  // The line the log was called from.
  uint16_t line;
  // The time the log was called.
  uint32_t time;
  // The message of the log.
  char *message;
} log_record_t;

/**
 * The formats of the logs available.
 */
typedef enum {
  // Comma seperated values
  CSV,
  // Prettified plain text
  PLAIN_TEXT,

} log_format_t;

/**
 * The destionation where the logs should be put.
 */
typedef enum {
  // Output the logs via the serial connection.
  UART,
} log_destination_t;

/**
 * Get the current log level of the logger.
 */
log_level_t logger_get_log_level();

/**
 * Initialize the logger.
 * @param level - The category that should be logged. All more restrictive
 * levels are included.
 * @param formal - The format you want your logs to be.
 * @param destination - Where do you want to log to.
 */
void logger_init(log_level_t level, log_format_t format,
                 log_destination_t destionation);

/**
 * Log the given log_entry
 * @param level - The log level of the message
 * @param module - The module the message is from
 * @param message - A printf like string of what the message should be. Add all
 * the values for the formatted string afterwards. The message can't exceed
 * [MAX_LOGGING_MESSAGE_LENGTH] characters.
 * @return whether the log was successful
 * @info Use the macro [LOGGER_LOG] for more convenience
 */
int logger_log(log_level_t level, log_module_t module, char *filename,
               uint16_t line_number, char *message, ...);

#define FILENAME                                                               \
  (__builtin_strrchr(__FILE__, '/') ? __builtin_strrchr(__FILE__, '/') + 1     \
                                    : __FILE__)

#define LOGGER_LOG(level, module, message, ...)                                \
  logger_log(level, module, (char *)FILENAME, (uint16_t)__LINE__, message,     \
             ##__VA_ARGS__)

/**
 * Function to log all the stored logs.
 * Should be called every [LOGGER_RUN_FREQ] ms.
 */
void logger_run();

#endif // LOGGER_H
