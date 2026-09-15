#pragma once
// Minimal logging-only adapter for ROS-independent native tests.  It does not
// implement ROS time, messages, publishers, parameters, or callbacks.
#include <cstdio>
#define ROS_INFO(...)  std::fprintf(stderr, "[INFO] " __VA_ARGS__), std::fputc('\n', stderr)
#define ROS_WARN(...)  std::fprintf(stderr, "[WARN] " __VA_ARGS__), std::fputc('\n', stderr)
#define ROS_ERROR(...) std::fprintf(stderr, "[ERROR] " __VA_ARGS__), std::fputc('\n', stderr)
#define ROS_DEBUG(...) do {} while (0)
#define ROS_WARN_THROTTLE(period, ...) ROS_WARN(__VA_ARGS__)
#define ROS_ERROR_THROTTLE(period, ...) ROS_ERROR(__VA_ARGS__)
