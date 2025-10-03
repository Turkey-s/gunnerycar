#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <string>
#include <sstream>
#include <thread>

namespace autofleet
{
const char* RED = "\033[31m";
const char* GREEN = "\033[32m";
const char* YELLOW = "\033[33m";
const char* BLUE = "\033[34m"; // 蓝色
const char* MAGENTA = "\033[35m"; // 酒红色
const char* CYAN = "\033[36m"; // 青蓝色
const char* WHITE = "\033[37m";
const char* RESET = "\033[0m";

// 具体级别的便捷宏
#define LOG_OUT_INFO(logger,format, ...)    RCLCPP_INFO(logger, "\033[37m" "[%s:%d %s]" format "\033[0m", __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define LOG_OUT_WARN(logger,format, ...)    RCLCPP_WARN(logger, "\033[34m" "[%s:%d %s]" format "\033[0m", __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define LOG_OUT_ERROR(logger,format, ...)   RCLCPP_ERROR(logger, "\033[31m" "[%s:%d %s]" format "\033[0m", __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define LOG_OUT_DEBUG(logger,format, ...)   RCLCPP_DEBUG(logger, "\033[33m" "[%s:%d %s]" format "\033[0m", __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define LOG_OUT_FATAL(logger,format, ...)   RCLCPP_FATAL(logger, "\033[35m" "[%s:%d %s]" format "\033[0m", __FILE__, __LINE__, __func__, ##__VA_ARGS__)

std::string thread_info()
{
    std::ostringstream thread_str;
    thread_str << "Thread ID: " << std::this_thread::get_id();
    return thread_str.str();
}
} // namespace autofleet

#endif // UTIL_HPP_