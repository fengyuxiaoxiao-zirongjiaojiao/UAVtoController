/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 10:32:00
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2024-12-28 13:36:02
 * @FilePath: /UAVtoController/src/main.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <signal.h>

#include "Application.hpp"

static void signalCapture(int signal)
{
    std::cout << __func__ << " " << signal << std::endl;
    Application::getInstance()->quit();
}

int main(int argc, char **argv)
{
    std::cout << "hello world" << std::endl;

    signal(SIGINT, signalCapture);
    Application app(argc, argv);
    if (app.init()) {
        app.exec();
    }

    std::cout << "Application exit done." << std::endl;
    return 0;
}