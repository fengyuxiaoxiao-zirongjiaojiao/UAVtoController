/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 10:32:00
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2025-01-05 15:22:05
 * @FilePath: /UAVtoController/src/main.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <signal.h>

#include "Application.hpp"

static void signalCapture(int signal)
{
    Logger::getInstance()->log(LOGLEVEL_INFO, "Application capture a signal:" + std::to_string(signal));
    Application::getInstance()->quit();
}

int main(int argc, char **argv)
{
    std::cout << "hello world" << std::endl;

    signal(SIGINT, signalCapture);
    Application app(argc, argv);
    if (app.init()) {
        Logger::getInstance()->log(LOGLEVEL_INFO, "Application init.");
        app.exec();
    }

    Logger::getInstance()->log(LOGLEVEL_INFO, "Application exit done.");
    return 0;
}