/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2025-01-05 14:29:38
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2025-01-05 16:56:10
 * @FilePath: /UAVtoController/src/Logger .hpp
 * @Description: 
 */
#ifndef __LOGGER_H_
#define __LOGGER_H_


#include <iostream>
#include <fstream>
#include <sstream>
#include <mutex>
#include <ctime>
#include <chrono>
#include <string>
#include <iomanip>
#include <thread>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <sstream>
#include <cassert>
#include <sys/stat.h>
#include <deque>

enum LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    FATAL
};

#define LOGLEVEL_DEBUG      DEBUG,__FILE__,__func__,__LINE__
#define LOGLEVEL_INFO       INFO,__FILE__,__func__,__LINE__
#define LOGLEVEL_WARNING    WARNING,__FILE__,__func__,__LINE__
#define LOGLEVEL_ERROR      ERROR,__FILE__,__func__,__LINE__
#define LOGLEVEL_FATAL      FATAL,__FILE__,__func__,__LINE__

class Logger {
public:
    ~Logger();

    static std::shared_ptr<Logger> _instance;
    static std::mutex _instanceMutex;
    static std::shared_ptr<Logger> getInstance(void);

    void setLogLevel(LogLevel level);
    int logLevel(void) { return _logLevel; }
    // 设置日志队列的最大大小
    void setMaxQueueSize(size_t size);
    void log(LogLevel level, const std::string& message);
    void log(LogLevel level, const char* file, const char* function, int line, const std::string& message);
    

private:
    std::string _logFilePath;
    LogLevel _logLevel;
    size_t _maxLogFileSize;
    size_t _maxQueueSize;
    std::mutex _mutex;
    std::deque<std::string> _logQueue;  // 使用deque，支持快速删除队首元素
    std::condition_variable _cv;
    std::atomic<bool> _shouldStop;
    std::thread _loggingThread;

    // 最大日志队列大小，默认是100条日志
    Logger(const std::string& logFilePath, size_t maxLogFileSize = 10 * 1024 * 1024, size_t maxQueueSize = 100);

    void _startLoggingThread();
    void _stopLoggingThread();
    void _processLogQueue();
    std::string _formatMessage(LogLevel level, const std::string& message);
    std::string _formatMessage(LogLevel level, const char* file, const char* function, int line, const std::string& message);
    std::string _currentDateTime();
    std::string _logLevelToString(LogLevel level);
    void _outputToConsole(const std::string& message);
    void _outputToFile(const std::string& message);
    bool _shouldRotateLogFile();
    void _rotateLogFile();
};


#endif