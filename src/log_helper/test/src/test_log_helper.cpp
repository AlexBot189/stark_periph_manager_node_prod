#define MODULE_NAME "test_log1"

#include <LogHelper.h>
#include <thread>
#include <chrono>

int main()
{
    int count = 0;
    while (true)
    {
        ++count;
        ECO_TRACE("trace log %d", count);
        ECO_TRACE_NEW("new trace log {}", count);
        ECO_DEBUG("debug log %d", count);
        ECO_DEBUG_NEW("new debug log {}", count);
        ECO_INFO("info log %d", count);
        ECO_INFO_NEW("new info log {}", count);
        ECO_WARN("warn log %d", count);
        ECO_WARN_NEW("new warn log {}", count);
        ECO_ERROR("error log %d", count);
        ECO_ERROR_NEW("new error log {}", count);
        ECO_FATAL("fatal log %d", count);
        ECO_FATAL_NEW("new fatal log {}", count);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}