#include <string.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include "LogHelper.h"
#include <map>
#include <iostream>

using namespace log_helper;

/**< 日志大小512kb, 0109：现阶段单个日志放开为10M */
const int MAX_SIZE = 10 * 1024 * 1024;
/**< 文件滚动数量，设置为1，最多生成2个文件 */
const int MAX_FILE_NUM = 1;

namespace log_helper
{
std::string EcoLogFormat(const char* format, ...)
{
    std::va_list args;
    va_start(args, format);
    auto len = std::vsnprintf(nullptr, 0, format, args);
    va_end(args);
    if (len <= 0)
    {
        return "";
    }

    std::string strBuf;
    strBuf.resize(len);
    va_start(args, format);
    std::vsnprintf(const_cast<char*>(strBuf.data()), strBuf.size() + 1, format, args);
    va_end(args);
    return std::move(strBuf);
}
}  // namespace log_helper

spdlog::level::level_enum GetConfigLogLevel()
{
    auto curLevel = getenv("ECO_LOG_LEVEL");
    if (curLevel)
    {
        std::map<std::string, spdlog::level::level_enum> name2level = {
            { "trace", spdlog::level::trace }, { "debug", spdlog::level::debug }, { "info", spdlog::level::info },
            { "warn", spdlog::level::warn }, { "err", spdlog::level::err }, { "fatal", spdlog::level::critical },
            { "off", spdlog::level::off },
        };

        std::string strLevel = curLevel;
        if (name2level.count(strLevel))
        {
            return name2level[strLevel];
        }

        if(strLevel.length() > 0 && std::isalnum(strLevel[0]))
        {
            int nLevel = atoi(strLevel.c_str());
            if(nLevel >= spdlog::level::trace && nLevel <= spdlog::level::off)
            {
                return static_cast<spdlog::level::level_enum>(nLevel);
            }
        }
    }

    return spdlog::level::info;
}

LogSinkType GetConfigLogSink()
{
    auto curSink = getenv("ECO_LOG_SINK");
    if (curSink)
    {
        std::map<std::string, LogSinkType> name2sink = {
            { "console", LogSinkType::LOG_SINK_STDOUT },
            { "file", LogSinkType::LOG_SINK_FILE },
            { "console_file", LogSinkType::LOG_SINK_STDOUT_FILE },
        };

        std::string strSink = curSink;
        if (name2sink.count(strSink))
        {
            return name2sink[strSink];
        }

        if(strSink.length() > 0 && std::isalnum(strSink[0]))
        {
            int nSink = atoi(strSink.c_str());
            if(nSink >= LogSinkType::LOG_SINK_STDOUT && nSink <= LogSinkType::LOG_SINK_STDOUT_FILE)
            {
                return static_cast<LogSinkType>(nSink);
            }
        }
    }

    return LogSinkType::LOG_SINK_STDOUT_FILE;
}

int GetConfigFlushInterval()
{
    auto curTime = getenv("ECO_LOG_TIME");
    if (curTime)
    {
        auto nTime = atoi(curTime);
        if (nTime > 0)
        {
            return nTime;
        }
    }

    return 5;
}

bool CreateDirectory(const std::string& path)
{
    int ret = mkdir(path.c_str(), 0777);
    if (ret == -1)
    {
        if (errno != EEXIST)
        {
            std::cerr << "Failed to create directory '" << path << "'. " << strerror(errno) << '\n';
            return false;
        }
    }

    std::cout << "Make log path " << path << " successfully" << std::endl;
    return true;
}

std::string GetConfigLogPath(const std::string& strName)
{
    std::string logPath;
    auto strHome = getenv("ECO_HOME");
    if (strHome)
    {
        logPath = strHome;
    }

    auto strLogPath = getenv("ECO_LOG_PATH");
    if (strLogPath)
    {
        logPath.append(static_cast<std::string>(strLogPath) + "/" + strName + "/");
    }
    else
    {
        logPath.append("/tmp/log/" + strName + "/");
    }

    // 自动创建日志目录
    CreateDirectory(logPath);

    return logPath + strName + ".txt";
}

int GetConfigLogFileSize()
{
    // 日志大小（kb）
    auto curSize = getenv("ECO_LOG_SIZE");
    if (curSize)
    {
        auto nSize = atoi(curSize);
        if (nSize > 0)
        {
            return nSize * 1024;
        }
    }

    return MAX_SIZE;
}

int GetConfigLogFileCount()
{
    auto curCount = getenv("ECO_LOG_COUNT");
    if (curCount)
    {
        auto nCount = atoi(curCount);
        if (nCount > 0)
        {
            return nCount;
        }
    }

    return MAX_FILE_NUM;
}

LoggerManager& LoggerManager::Instance()
{
    static LoggerManager loggerManager;
    return loggerManager;
}

LoggerManager::LoggerManager()
{

}

void LoggerManager::Create(const std::string& strName, LogSinkType sinkType, LogSeverity logSeverity)
{
    if (spdlog::get(strName))
    {
        return;
    }

    if (sinkType == LOG_SINK_DEFAULT)
    {
        sinkType = GetConfigLogSink();
    }

    std::vector<spdlog::sink_ptr> sinks;
    if (sinkType == LOG_SINK_STDOUT || sinkType == LOG_SINK_STDOUT_FILE)
    {
        auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        sinks.emplace_back(consoleSink);
    }

    if (sinkType == LOG_SINK_FILE || sinkType == LOG_SINK_STDOUT_FILE)
    {
        auto fileSink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(GetConfigLogPath(strName),
            GetConfigLogFileSize(), GetConfigLogFileCount());
        sinks.emplace_back(fileSink);
    }

    auto logger = std::make_shared<spdlog::logger>(strName, sinks.begin(), sinks.end());
    // 注册logger对象
    spdlog::register_logger(logger);

    if (logSeverity == LOG_SEVERITY_DEFAULT)
    {
        logger->set_level(GetConfigLogLevel());
    }
    else
    {
        logger->set_level(static_cast<spdlog::level::level_enum>(logSeverity));
    }
    logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%s:%# %!] %v");
    spdlog::flush_every(std::chrono::seconds(GetConfigFlushInterval()));
    // 错误日志立即刷新
    logger->flush_on(spdlog::level::err);
}

void LoggerManager::Flush(const std::string& strName)
{
    auto pLogger = spdlog::get(strName);
    if (pLogger)
    {
        pLogger->flush();
    }
}

std::shared_ptr<spdlog::logger> LoggerManager::GetNativeLogger(const std::string& loggerName)
{
    return spdlog::get(loggerName);
}

EcoLogger::EcoLogger(const std:: string& name)
    : m_name(name)
{
    LoggerManager::Instance().Create(m_name);
}

EcoLogger::~EcoLogger()
{
}

std::shared_ptr<spdlog::logger> EcoLogger::GetNativeLogger()
{
    return LoggerManager::Instance().GetNativeLogger(m_name);
}