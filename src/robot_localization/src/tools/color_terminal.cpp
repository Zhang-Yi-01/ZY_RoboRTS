/*
 * @Description: 终端彩色输出
 */

#include "../../include/tools/color_terminal.hpp"
// c++
#include <string>
#include <iostream>

namespace robot_localization
{

    /**
     * @brief 彩色终端输出--节点级别
     * @note 蓝色
     * @todo
     **/
    void ColorTerminal::ColorNodeInfo(const std::string str)
    {
        std::cout << std::endl
                  << fontColorBlueBold << str << fontColorReset << std::endl
                  << std::endl;
    }

    /**
     * @brief 彩色终端输出--任务管理级别
     * @note 黄色
     * @todo
     **/
    void ColorTerminal::ColorFlowInfo(const std::string str)
    {
        std::cout << std::endl
                  << fontColorYellowBold << str << fontColorReset << std::endl
                  << std::endl;
    }

    /**
     * @brief 彩色终端输出--算法实现级别
     * @note 绿色
     * @todo
     **/
    void ColorTerminal::ColorConcreteInfo(const std::string str)
    {
        std::cout << std::endl
                  << fontColorGreenBold << str << fontColorReset << std::endl
                  << std::endl;
        ;
    }

    /**
     * @brief 彩色终端输出--算法实现级别
     * @note 绿色
     * @todo
     **/
    void ColorTerminal::ColorConcreteDebug(const std::string str)
    {
        std::cout << std::endl
                  << fontColorWhiteBold << str << fontColorReset << std::endl
                  << std::endl;
        ;
    }

    void ColorTerminal::green(const std::string str)
    {

        std::cout << fontColorGreenBold << str << fontColorReset << std::endl;
    }
    void ColorTerminal::red(const std::string str)
    {

        std::cout << fontColorRedBold << str << fontColorReset << std::endl;
    }
    void ColorTerminal::white(const std::string str)
    {

        std::cout << fontColorWhiteBold << str << fontColorReset << std::endl;
    }
    void ColorTerminal::yellow(const std::string str)
    {

        std::cout << fontColorYellowBold << str << fontColorReset << std::endl;
    }
    void ColorTerminal::blue(const std::string str)
    {

        std::cout << fontColorBlueBold << str << fontColorReset << std::endl;
    }
    void ColorTerminal::cyan(const std::string str)
    {

        std::cout << fontColorCyanBold << str << fontColorReset << std::endl;
    }
    void ColorTerminal::green(const std::string str,bool flag)
    {   
        if(flag)
        std::cout << fontColorGreenBold << str << fontColorReset << std::endl;
    }
    void ColorTerminal::red(const std::string str,bool flag)
    {
        if(flag)
        std::cout << fontColorRedBold << str << fontColorReset << std::endl;
    }
    void ColorTerminal::white(const std::string str,bool flag)
    {
        if(flag)
        std::cout << fontColorWhiteBold << str << fontColorReset << std::endl;
    }
    void ColorTerminal::yellow(const std::string str,bool flag)
    {
        if(flag)
        std::cout << fontColorYellowBold << str << fontColorReset << std::endl;
    }
    void ColorTerminal::blue(const std::string str,bool flag)
    {
        if(flag)
        std::cout << fontColorBlueBold << str << fontColorReset << std::endl;
    }
    void ColorTerminal::cyan(const std::string str,bool flag)
    {
        if(flag)
        std::cout << fontColorCyanBold << str << fontColorReset << std::endl;
    }
} // namespace robot_localization
