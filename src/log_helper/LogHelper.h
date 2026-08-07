#ifndef _LOG_HELPER_H_
#define _LOG_HELPER_H_

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include <spdlog/spdlog.h>
#include <sstream>

#ifdef MODULE_NAME
#define LOGGING_NAME MODULE_NAME
#else
#define LOGGING_NAME "eco_default_log"
#endif

namespace log_helper
{
/**< 日志等级 */
enum LogSeverity
{
    LOG_SEVERITY_DEFAULT = -1,
    LOG_SEVERITY_TRACE = 0,
    LOG_SEVERITY_DEBUG = 1,
    LOG_SEVERITY_INFO = 2,
    LOG_SEVERITY_WARN = 3,
    LOG_SEVERITY_ERROR = 4,
    LOG_SEVERITY_FATAL = 5,
    LOG_SEVERITY_OFF = 6
};

enum LogSinkType
{
    LOG_SINK_DEFAULT = -1,
    LOG_SINK_STDOUT = 0,
    LOG_SINK_FILE = 1,
    LOG_SINK_STDOUT_FILE = 2
};

class LoggerManager
{
public:
    static LoggerManager& Instance();

    void Create(const std::string& strName, LogSinkType sinkType = LOG_SINK_DEFAULT, LogSeverity logSeverity = LOG_SEVERITY_DEFAULT);

    void Flush(const std::string& strName);

    std::shared_ptr<spdlog::logger> GetNativeLogger(const std::string& loggerName);

private:
    LoggerManager();
};

class EcoLogger {
public:
    explicit
    EcoLogger(const std::string& name);

    ~EcoLogger();

    std::shared_ptr<spdlog::logger> GetNativeLogger();
private:
    std::string m_name;
};

inline std::shared_ptr<spdlog::logger> GetDefaultLogger()
{
    static EcoLogger defaultLogger(LOGGING_NAME);
    return defaultLogger.GetNativeLogger();
}

std::string EcoLogFormat(const char* format, ...);

}  // namespace log_helper

/* 新日志接口(采用{}占位符) */
#define ECO_TRACE_NEW(...)  SPDLOG_LOGGER_TRACE(log_helper::GetDefaultLogger(), __VA_ARGS__)

#define ECO_DEBUG_NEW(...)  SPDLOG_LOGGER_DEBUG(log_helper::GetDefaultLogger(), __VA_ARGS__)

#define ECO_INFO_NEW(...)  SPDLOG_LOGGER_INFO(log_helper::GetDefaultLogger(), __VA_ARGS__)

#define ECO_WARN_NEW(...)  SPDLOG_LOGGER_WARN(log_helper::GetDefaultLogger(), __VA_ARGS__)

#define ECO_ERROR_NEW(...)  SPDLOG_LOGGER_ERROR(log_helper::GetDefaultLogger(), __VA_ARGS__)

#define ECO_FATAL_NEW(...)  SPDLOG_LOGGER_CRITICAL(log_helper::GetDefaultLogger(), __VA_ARGS__)

/* 原日志接口 */
#define ECO_TRACE(...)  ECO_TRACE_NEW(log_helper::EcoLogFormat(__VA_ARGS__))

#define ECO_DEBUG(...)  ECO_DEBUG_NEW(log_helper::EcoLogFormat(__VA_ARGS__))

#define ECO_INFO(...)  ECO_INFO_NEW(log_helper::EcoLogFormat(__VA_ARGS__))

#define ECO_WARN(...)  ECO_WARN_NEW(log_helper::EcoLogFormat(__VA_ARGS__))

#define ECO_ERROR(...)  ECO_ERROR_NEW(log_helper::EcoLogFormat(__VA_ARGS__))

#define ECO_FATAL(...)  ECO_FATAL_NEW(log_helper::EcoLogFormat(__VA_ARGS__))

#define LOG_FLUSH() log_helper::GetDefaultLogger()->flush()

/**< 2. 增加条件日志打印 */
#define ECO_DEBUG_COND(cond, ...)   \
    do                              \
    {                               \
        if (cond)                   \
        {                           \
            ECO_DEBUG_NEW(__VA_ARGS__); \
        }                           \
    } while (0);

#define ECO_INFO_COND(cond, ...)   \
    do                             \
    {                              \
        if (cond)                  \
        {                          \
            ECO_INFO_NEW(__VA_ARGS__); \
        }                          \
    } while (0);

#define ECO_WARN_COND(cond, ...)   \
    do                             \
    {                              \
        if (cond)                  \
        {                          \
            ECO_WARN_NEW(__VA_ARGS__); \
        }                          \
    } while (0);

#define ECO_ERROR_COND(cond, ...)   \
    do                              \
    {                               \
        if (cond)                   \
        {                           \
            ECO_ERROR_NEW(__VA_ARGS__); \
        }                           \
    } while (0);

#define ECO_FATAL_COND(cond, ...)   \
    do                              \
    {                               \
        if (cond)                   \
        {                           \
            ECO_FATAL_NEW(__VA_ARGS__); \
        }                           \
    } while (0);

/**< 3. 循环单次打印 */
#define ECO_LOG_ONCE(...)          \
    do                             \
    {                              \
        static bool hit = false;   \
        if (!hit)                  \
        {                          \
            hit = true;            \
            ECO_INFO_NEW(__VA_ARGS__); \
        }                          \
    } while (0);

/**< 4. 周期性定时打印日志: period(ms) */
#define ECO_LOG_THROTTLE(period_ms, ...)                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        static double log_throttle_last_hit = 0;                                                                       \
        double log_throttle_now =                                                                                      \
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()) \
                .count();                                                                                              \
        double duration = log_throttle_now - log_throttle_last_hit;                                                    \
        if (duration > period_ms)                                                                                      \
        {                                                                                                              \
            log_throttle_last_hit = log_throttle_now;                                                                  \
            ECO_INFO_NEW(__VA_ARGS__);                                                                                     \
        }                                                                                                              \
    } while (0);

/**< 5. 过滤打印int */
#define ECO_FILTER_LOG(value, ...)          \
    do                                      \
    {                                       \
        static int last_value = 0xffffffff; \
        if (last_value != value)            \
        {                                   \
            last_value = value;             \
            ECO_DEBUG_NEW(__VA_ARGS__);         \
        }                                   \
    } while (0);

/**< 6. 降频过滤打印int : 给定时间间隔--> min, max */
#define ECO_REDUCE_LOG(value, min_ms, max_ms, ...)                                                                     \
    do                                                                                                                 \
    {                                                                                                                  \
        static double last_log_time = 0;                                                                               \
        static double interval_ms = min_ms;                                                                            \
        static int last_value = 0xffffffff;                                                                            \
        static bool time_set_error = min_ms > max_ms;                                                                  \
        double log_time_now =                                                                                          \
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()) \
                .count();                                                                                              \
        if (time_set_error)                                                                                            \
        {                                                                                                              \
            double duration = log_time_now - last_log_time;                                                            \
            if (duration > 4000)                                                                                       \
            {                                                                                                          \
                ECO_ERROR_NEW("Invalid min/max time set, min = {} > max = {}", min_ms, max_ms);                            \
                last_log_time = log_time_now;                                                                          \
            }                                                                                                          \
        }                                                                                                              \
        else                                                                                                           \
        {                                                                                                              \
            if (last_value != value)                                                                                   \
            {                                                                                                          \
                last_value = value;                                                                                    \
                interval_ms = min_ms;                                                                                  \
                ECO_DEBUG_NEW(__VA_ARGS__);                                                                                \
                last_log_time = log_time_now;                                                                          \
            }                                                                                                          \
            else                                                                                                       \
            {                                                                                                          \
                double duration = log_time_now - last_log_time;                                                        \
                if (duration > interval_ms)                                                                            \
                {                                                                                                      \
                    interval_ms = interval_ms + 1000;                                                                  \
                    if (interval_ms > max_ms)                                                                          \
                    {                                                                                                  \
                        interval_ms = max_ms;                                                                          \
                    }                                                                                                  \
                    ECO_DEBUG_NEW(__VA_ARGS__);                                                                            \
                    last_log_time = log_time_now;                                                                      \
                }                                                                                                      \
            }                                                                                                          \
        }                                                                                                              \
                                                                                                                       \
    } while (0);

/**< 7. 流式打印 */
#define ECO_DEBUG_STREAM(stream_arg)       \
    do                                     \
    {                                      \
        std::stringstream ss;              \
        ss << stream_arg;                  \
        ECO_DEBUG_NEW("{}", ss.str().c_str()); \
    } while (0)

#define ECO_INFO_STREAM(stream_arg)       \
    do                                    \
    {                                     \
        std::stringstream ss;             \
        ss << stream_arg;                 \
        ECO_INFO_NEW("{}", ss.str().c_str()); \
    } while (0)

#define ECO_WARN_STREAM(stream_arg)       \
    do                                    \
    {                                     \
        std::stringstream ss;             \
        ss << stream_arg;                 \
        ECO_WARN_NEW("{}", ss.str().c_str()); \
    } while (0)

#define ECO_ERROR_STREAM(stream_arg)       \
    do                                     \
    {                                      \
        std::stringstream ss;              \
        ss << stream_arg;                  \
        ECO_ERROR_NEW("{}", ss.str().c_str()); \
    } while (0)

#define ECO_FATAL_STREAM(stream_arg)       \
    do                                     \
    {                                      \
        std::stringstream ss;              \
        ss << stream_arg;                  \
        ECO_FATAL_NEW("{}", ss.str().c_str()); \
    } while (0)

#endif