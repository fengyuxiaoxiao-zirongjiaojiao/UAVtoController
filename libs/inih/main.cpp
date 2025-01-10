#include <iostream>
#include <fstream>
#include <string>
#include "INIReader.h"

// 函数声明
void writeINI(const std::string &filename, const INIReader &reader);

int main() {
    // 创建INIReader对象并加载配置文件
    INIReader reader("config.ini");

    // 检查文件是否被正确加载
    if (reader.ParseError() < 0) {
        std::cerr << "无法加载INI文件" << std::endl;
        return 1;
    }

    // 读取数据库相关参数
    std::string host = reader.Get("database", "host", "default_host");
    int port = reader.GetInteger("database", "port", 5432);
    std::string username = reader.Get("database", "username", "default_user");
    std::string password = reader.Get("database", "password", "default_password");

    // 读取显示设置
    std::string resolution = reader.Get("settings", "resolution", "800x600");
    bool fullscreen = reader.GetBoolean("settings", "fullscreen", false);

    // 输出读取到的参数
    std::cout << "Database Configuration:" << std::endl;
    std::cout << "Host: " << host << std::endl;
    std::cout << "Port: " << port << std::endl;
    std::cout << "Username: " << username << std::endl;
    std::cout << "Password: " << password << std::endl;

    std::cout << "\nSettings Configuration:" << std::endl;
    std::cout << "Resolution: " << resolution << std::endl;
    std::cout << "Fullscreen: " << (fullscreen ? "true" : "false") << std::endl;

    // 修改一些配置
    host = "127.0.0.1";
    port = 3306;
    resolution = "2560x1440";
    fullscreen = true;

    // 输出修改后的参数
    std::cout << "\nModifying Configuration..." << std::endl;
    std::cout << "New Host: " << host << std::endl;
    std::cout << "New Port: " << port << std::endl;
    std::cout << "New Resolution: " << resolution << std::endl;
    std::cout << "New Fullscreen: " << (fullscreen ? "true" : "false") << std::endl;

    // 创建一个新的INIReader以更新文件
    writeINI("config.ini", reader); // 写回更新的配置

    // 返回成功标志
    return 0;
}

// 写入新的配置到INI文件
void writeINI(const std::string &filename, const INIReader &reader) {
    std::ofstream iniFile(filename);

    if (!iniFile.is_open()) {
        std::cerr << "无法打开INI文件进行写入!" << std::endl;
        return;
    }

    // 写入数据库配置
    iniFile << "[database]" << std::endl;
    iniFile << "host = 127.0.0.1" << std::endl;
    iniFile << "port = 3306" << std::endl;
    iniFile << "username = " << reader.Get("database", "username", "default_user") << std::endl;
    iniFile << "password = " << reader.Get("database", "password", "default_password") << std::endl;

    // 写入显示设置
    iniFile << "[settings]" << std::endl;
    iniFile << "resolution = 2560x1440" << std::endl;
    iniFile << "fullscreen = true" << std::endl;

    iniFile.close();
    std::cout << "配置已更新并写回到INI文件!" << std::endl;
}

