/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2025-01-05 14:29:45
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2025-01-05 17:33:18
 * @FilePath: /UAVtoController/src/Logger .cpp
 * @Description: 
 */
#include "Logger.hpp"
#include <sys/stat.h>
#include <unistd.h>

std::shared_ptr<Logger> Logger::_instance = nullptr;
std::mutex Logger::_instanceMutex;

Logger::Logger(const std::string& logFilePath, size_t maxLogFileSize, size_t maxQueueSize)
    : _logFilePath(logFilePath), _logLevel(INFO), _maxLogFileSize(maxLogFileSize), _maxQueueSize(maxQueueSize), _shouldStop(false) {
    _startLoggingThread();
}

Logger::~Logger() {
    _stopLoggingThread();
}

std::shared_ptr<Logger> Logger::getInstance(void)
{
    if (!_instance) {
        std::lock_guard<std::mutex> lock(_instanceMutex);
        if (!_instance) {
            auto now = std::chrono::system_clock::now();
            std::time_t timeNow = std::chrono::system_clock::to_time_t(now);
            std::tm tm = *std::localtime(&timeNow);

            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
            
            const char* home_dir = std::getenv("HOME");
            std::string path = std::string(home_dir) + "/uavToController";  // 要创建的文件夹路径
            // 检查目录是否已经存在
            struct stat info;
            if (! (stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR)) ) {
                if (mkdir(path.c_str(), 0755) == 0) {
                    std::cout << "Logger 文件夹创建成功: " << path << std::endl;
                } else {
                    std::cout << "Logger 创建文件夹失败!" << std::endl;
                }
            } else {
                std::cout << "Logger 文件夹已存在: " << path << std::endl;
            }
            
            std::string logFilePath =  path + "/uavToController_" + oss.str() + ".log";
            _instance = std::shared_ptr<Logger>(new Logger(logFilePath));
        }
    }
    return _instance;
}

void Logger::setLogLevel(LogLevel level) {
    _logLevel = level;
}

// 设置日志队列的最大大小
void Logger::setMaxQueueSize(size_t size) {
    _maxQueueSize = size;
}

void Logger::log(LogLevel level, const std::string& message) {
    if (level >= _logLevel) {
        std::string logMessage = _formatMessage(level, message);

        {
            std::lock_guard<std::mutex> lock(_mutex);

            // 如果队列已满，丢弃最老的日志
            if (_logQueue.size() >= _maxQueueSize) {
                _logQueue.pop_front(); // 丢弃队列中的最老日志
            }

            _logQueue.push_back(logMessage);
        }
        _cv.notify_one();
    }
}

void Logger::log(LogLevel level, const char* file, const char* function, int line, const std::string& message)
{
    if (level >= _logLevel) {
        std::string logMessage = _formatMessage(level, file, function, line, message);

        {
            std::lock_guard<std::mutex> lock(_mutex);

            // 如果队列已满，丢弃最老的日志
            if (_logQueue.size() >= _maxQueueSize) {
                _logQueue.pop_front(); // 丢弃队列中的最老日志
            }

            _logQueue.push_back(logMessage);
        }
        _cv.notify_one();
    }
}


void Logger::_startLoggingThread() {
    _loggingThread = std::thread(&Logger::_processLogQueue, this);
}

void Logger::_stopLoggingThread() {
    _shouldStop = true;
    _cv.notify_all();
    if (_loggingThread.joinable()) {
        _loggingThread.join();
    }
}

void Logger::_processLogQueue() {
    while (!_shouldStop) {
        std::unique_lock<std::mutex> lock(_mutex);
        _cv.wait(lock, [this] { return !_logQueue.empty() || _shouldStop; });

        while (!_logQueue.empty()) {
            std::string logMessage = _logQueue.front();
            _logQueue.pop_front();
            _outputToConsole(logMessage);
            _outputToFile(logMessage);
        }
    }
}

std::string Logger::_formatMessage(LogLevel level, const std::string& message) {
    std::ostringstream ss;
    ss << "[" << _currentDateTime() << "] ";
    ss << "[" << _logLevelToString(level) << "] ";
    ss << message;
    return ss.str();
}

std::string Logger::_formatMessage(LogLevel level, const char* file, const char* function, int line, const std::string& message)
{
    std::ostringstream ss;
    ss << "[" << _currentDateTime() << "] ";
    ss << "[" << _logLevelToString(level) << "] ";
    ss << "at [" << std::string(file) << ":" << std::string(function) << ":" << line << "] ";
    ss << message;
    return ss.str();
}

std::string Logger::_currentDateTime() {
    auto now = std::chrono::system_clock::now();
    std::time_t timeNow = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&timeNow);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

std::string Logger::_logLevelToString(LogLevel level) {
    switch (level) {
        case DEBUG:   return "DEBUG";
        case INFO:    return "INFO";
        case WARNING: return "WARNING";
        case ERROR:   return "ERROR";
        case FATAL:   return "FATAL";
        default:      return "UNKNOWN";
    }
}

void Logger::_outputToConsole(const std::string& message) {
    std::cout << message << std::endl;
}

void Logger::_outputToFile(const std::string& message) {
    if (_shouldRotateLogFile()) {
        _rotateLogFile();
    }

    std::ofstream logFile(_logFilePath, std::ios::app);
    if (logFile.is_open()) {
        logFile << message << std::endl;
        logFile.close();
    }
}

bool Logger::_shouldRotateLogFile() {
    struct stat fileStatus;
    if (stat(_logFilePath.c_str(), &fileStatus) == 0) {
        return fileStatus.st_size >= _maxLogFileSize;
    }
    return false;
}

void Logger::_rotateLogFile() {
    static int rotationCount = 0;
    rotationCount++;

    // 重命名现有日志文件
    std::ostringstream oss;
    oss << _logFilePath << "." << rotationCount;
    std::string backupFilePath = oss.str();

    std::rename(_logFilePath.c_str(), backupFilePath.c_str());
}

