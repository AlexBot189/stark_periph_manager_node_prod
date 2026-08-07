// Copyright(c) 2015-present, Gabi Melman & spdlog contributors.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)

// spdlog main header file.
// see example.cpp for usage example

#ifndef _MY_LOGGER_INITIALIZER_
#define _MY_LOGGER_INITIALIZER_

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <sys/stat.h>
#include <sys/types.h>

#ifdef _WIN32
#include <spdlog/sinks/wincolor_sink.h>
#else
#include <spdlog/sinks/ansicolor_sink.h>
#endif

#include <spdlog/details/fmt_helper.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>

namespace spdlog
{

// Full info formatter
// pattern: [%Y-%m-%d %H:%M:%S.%f] [%t] [%^%l%$] [%s:%#] %v
class FullFormatter final : public spdlog::custom_flag_formatter
{
public:
    explicit FullFormatter()
    {
    }

    std::unique_ptr<custom_flag_formatter> clone() const override
    {
        auto formatter = spdlog::details::make_unique<FullFormatter>();
        formatter->cache_timestamp_ = cache_timestamp_;
        return std::move(formatter);
    }

    void format(const spdlog::details::log_msg &msg, const std::tm &tm_time, spdlog::memory_buf_t &dest) override
    {
        using std::chrono::duration_cast;
        using std::chrono::microseconds;
        using std::chrono::seconds;

        // cache the date/time part for the next second.
        auto duration = msg.time.time_since_epoch();
        auto secs = duration_cast<seconds>(duration);

        if (cache_timestamp_ != secs || cached_datetime_.size() == 0)
        {
            cached_datetime_.clear();
            cached_datetime_.push_back('[');
            spdlog::details::fmt_helper::append_int(tm_time.tm_year + 1900, cached_datetime_);
            cached_datetime_.push_back('-');

            spdlog::details::fmt_helper::pad2(tm_time.tm_mon + 1, cached_datetime_);
            cached_datetime_.push_back('-');

            spdlog::details::fmt_helper::pad2(tm_time.tm_mday, cached_datetime_);
            cached_datetime_.push_back(' ');

            spdlog::details::fmt_helper::pad2(tm_time.tm_hour, cached_datetime_);
            cached_datetime_.push_back(':');

            spdlog::details::fmt_helper::pad2(tm_time.tm_min, cached_datetime_);
            cached_datetime_.push_back(':');

            spdlog::details::fmt_helper::pad2(tm_time.tm_sec, cached_datetime_);
            cached_datetime_.push_back('.');

            cache_timestamp_ = secs;
        }
        dest.append(cached_datetime_.begin(), cached_datetime_.end());

        auto micros = spdlog::details::fmt_helper::time_fraction<microseconds>(msg.time);
        spdlog::details::fmt_helper::pad6(static_cast<uint32_t>(micros.count()), dest);
        dest.push_back(']');
        dest.push_back(' ');

        // thread id
        dest.push_back('[');
        spdlog::details::fmt_helper::append_int(msg.thread_id, dest);
        dest.push_back(']');
        dest.push_back(' ');

        // append logger name if exists
        if (msg.logger_name.size() > 0)
        {
            dest.push_back('[');
            spdlog::details::fmt_helper::append_string_view(msg.logger_name, dest);
            dest.push_back(']');
            dest.push_back(' ');
        }

        dest.push_back('[');
        // wrap the level name with color
        msg.color_range_start = dest.size();
        // fmt_helper::append_string_view(level::to_c_str(msg.level), dest);
        spdlog::details::fmt_helper::append_string_view(spdlog::level::to_string_view(msg.level), dest);
        msg.color_range_end = dest.size();
        dest.push_back(']');
        dest.push_back(' ');

        // add source location if present
        if (!msg.source.empty())
        {
            dest.push_back('[');
            const char *filename = GetBaseName(msg.source.filename);
            spdlog::details::fmt_helper::append_string_view(filename, dest);
            dest.push_back(':');
            spdlog::details::fmt_helper::append_int(msg.source.line, dest);
            dest.push_back(']');
            dest.push_back(' ');
        }
        // fmt_helper::append_string_view(msg.msg(), dest);
        spdlog::details::fmt_helper::append_string_view(msg.payload, dest);
    }

private:
    static const char *GetBaseName(const char *filename)
    {
        const char *rv = std::strrchr(filename, spdlog::details::os::folder_seps[0]);
        return rv != nullptr ? rv + 1 : filename;
    }

private:
    std::chrono::seconds cache_timestamp_{0};
    spdlog::memory_buf_t cached_datetime_;
};

class Logger
{
public:
    Logger(const std::string &nodeName, spdlog::level::level_enum initLogLevel)
    {
        InitLogger(nodeName, initLogLevel);
    }

    void InitLogger(const std::string &nodeName,
                    spdlog::level::level_enum initLogLevel = spdlog::level::err,
                    std::size_t maxSize = 5 * 1024 * 1024, ///< 日志大小512kb
                    std::size_t maxFiles = 1,
                    std::chrono::seconds flushInterval = std::chrono::seconds(5))
    {
        // std::cout << "InitLogger " << nodeName << std::endl;
        std::string logDir = "/mnt/UDISK/robot/log/";
        if (access(logDir.c_str(), 0) == -1)
        {
            int flag = mkdir(logDir.c_str(), 0777);
            if (flag == 0)
            {
                std::cout << "make successfully" << std::endl;
            }
            else
            {
                std::cout << "make error" << std::endl;
            }
        }

        if (spdlog::get(nodeName))
        {
            // std::cout << "logger " << nodeName << "aleardy exist" << std::endl;
            return;
        }
        // Set format [%Y-%m-%d %H:%M:%S.%f] [%t] [%^%l%$] [%s:%#] %v
        auto formatter = spdlog::details::make_unique<spdlog::pattern_formatter>();
        formatter->add_flag<FullFormatter>('~').set_pattern("%~");

        char *name = std::getenv("SPDLOG_DEFAULT_LOGGER");
        std::string defaultLoggerName = (name == nullptr) ? "" : name;
        // std::cout << "register logger " << nodeName  << " default logger " << defaultLoggerName << std::endl;
        std::shared_ptr<spdlog::logger> defaultLogger_;

        if (!defaultLoggerName.empty() && !std::strcmp(defaultLoggerName.c_str(), "console"))
        {
#ifdef _WIN32
            auto color_sink = std::make_shared<spdlog::sinks::wincolor_stdout_sink_mt>();
#else
            auto color_sink = std::make_shared<spdlog::sinks::ansicolor_stdout_sink_mt>();
#endif
            defaultLogger_ = std::make_shared<spdlog::logger>(nodeName, std::move(color_sink));
            spdlog::register_logger(defaultLogger_);
        }
        else
        {
            defaultLogger_ = spdlog::rotating_logger_mt(nodeName, "/mnt/UDISK/robot/log/" + nodeName + ".txt", maxSize, maxFiles);
        }

        defaultLogger_->set_formatter(std::move(formatter));

        // Set log level
        const char *logLevel = std::getenv("SPDLOG_FIXED_LOG_LEVEL");
        if (logLevel != nullptr)
        {
            SPDLOG_INFO("Load fixed log level from environment: {}",
                        spdlog::level::to_string_view(spdlog::level::from_str(logLevel)).data());
            defaultLogger_->set_level(spdlog::level::from_str(logLevel));
        }
        else
        {
            logLevel = std::getenv("SPDLOG_INITIAL_LOG_LEVEL");
            if (logLevel != nullptr)
            {
                SPDLOG_INFO("Load initial log level from environment: {}",
                            spdlog::level::to_string_view(spdlog::level::from_str(logLevel)).data());
                defaultLogger_->set_level(spdlog::level::from_str(logLevel));
            }
            else
            {
                defaultLogger_->set_level(initLogLevel);
            }
        }
        // 每隔flushInterval秒写一次log
        spdlog::flush_every(std::chrono::seconds(flushInterval));

        // 遇到错误日志级别以上立刻刷新日志
        defaultLogger_->flush_on(spdlog::level::err);
    }
};

template <typename T>
inline void logger_wrapper_call(std::shared_ptr<spdlog::logger> logger, source_loc source, level::level_enum lvl, const T &msg)
{
    if (!logger)
    {
        return;
    }
    logger->log(source, lvl, msg);
}

template <typename FormatString, typename... Args>
inline void logger_wrapper_call(std::shared_ptr<spdlog::logger> logger, source_loc source, level::level_enum lvl, const FormatString &fmt, const Args &...args)
{
    if (!logger)
    {
        // std::cout << "logging failed" << std::endl;
        return;
    }
    logger->log(source, lvl, fmt, args...);
}

#define LOGGER_WRAPPER_CALL(logger, level, ...) logger_wrapper_call(logger, spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, level, __VA_ARGS__)

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_TRACE
#define LOGGER_WRAPPER_TRACE(loggerName, ...) LOGGER_WRAPPER_CALL(spdlog::get(loggerName), spdlog::level::trace, __VA_ARGS__)
#else
#define LOGGER_WRAPPER_TRACE(loggerName, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_DEBUG
#define LOGGER_WRAPPER_DEBUG(loggerName, ...) LOGGER_WRAPPER_CALL(spdlog::get(loggerName), spdlog::level::debug, __VA_ARGS__)
#else
#define LOGGER_WRAPPER_DEBUG(loggerName, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_INFO
#define LOGGER_WRAPPER_INFO(loggerName, ...) LOGGER_WRAPPER_CALL(spdlog::get(loggerName), spdlog::level::info, __VA_ARGS__)
#else
#define LOGGER_WRAPPER_INFO(loggerName, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_WARN
#define LOGGER_WRAPPER_WARN(loggerName, ...) LOGGER_WRAPPER_CALL(spdlog::get(loggerName), spdlog::level::warn, __VA_ARGS__)
#else
#define LOGGER_WRAPPER_WARN(loggerName, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_ERROR
#define LOGGER_WRAPPER_ERROR(loggerName, ...) LOGGER_WRAPPER_CALL(spdlog::get(loggerName), spdlog::level::err, __VA_ARGS__)
#else
#define LOGGER_WRAPPER_ERROR(loggerName, ...) (void)0
#endif

} // namespace spdlog

#endif // define _MY_LOGGER_INITIALIZER_