/*
 * @Description: 终端彩色输出
 * @Author: ZY、genshin_zy
 * @Date: 2022.10.24
 */
#ifndef COLOR_TERMINAL_HPP
#define COLOR_TERMINAL_HPP

// c++
#include <string>
// global_defination
#include "ros/package.h"
#include <yaml-cpp/yaml.h>

#define if_show_working_info YAML::LoadFile(ros::package::getPath("robot_localization") + "/config/user_setting.yaml")["if_show_working_info"].as<bool>()
/***
 * @example 例如，提示绿色字体，直接用green_info("str"),如果由user_setting.yaml里设置打印此部分调试信息，就打印
*/
#define green_info(str)  ColorTerminal::green(str,if_show_working_info)
#define white_info(str)  ColorTerminal::white(str,if_show_working_info)
#define cyan_info(str)   ColorTerminal::cyan(str,if_show_working_info)
#define blue_info(str)   ColorTerminal::blue(str,if_show_working_info)
#define yellow_info(str) ColorTerminal::yellow(str,if_show_working_info)
#define red_info(str)    ColorTerminal::red(str,if_show_working_info)

namespace robot_localization
{

/*终端彩色字体*/
#define fontColorReset "\033[0m"
#define fontColorBlack "\033[30m"
#define fontColorRed "\033[31m"
#define fontColorGreen "\033[32m"
#define fontColorYellow "\033[33m"
#define fontColorBlue "\033[34m"
#define fontColorMagenta "\033[35m"
#define fontColorCyan "\033[36m"
#define fontColorWhite "\033[37m"
#define fontColorBlackBold "\033[1m\033[30m"
#define fontColorRedBold "\033[1m\033[31m"
#define fontColorGreenBold "\033[1m\033[32m"
#define fontColorYellowBold "\033[1m\033[33m"
#define fontColorBlueBold "\033[1m\033[34m"
#define fontColorMagentaBold "\033[1m\033[35m"
#define fontColorCyanBold "\033[1m\033[36m"
#define fontColorWhiteBold "\033[1m\033[37m"

        class ColorTerminal
        {
        public:
                static void ColorNodeInfo(const std::string str);
                static void ColorFlowInfo(const std::string str);
                static void ColorConcreteInfo(const std::string str);
                static void ColorConcreteDebug(const std::string str);

                static void green(const std::string str,bool flag);
                static void red(const std::string str,bool flag);
                static void yellow(const std::string str,bool flag);
                static void white(const std::string str,bool flag);
                static void blue(const std::string str,bool flag);
                static void cyan(const std::string str,bool flag);

                static void green(const std::string str);
                static void red(const std::string str);
                static void yellow(const std::string str);
                static void white(const std::string str);
                static void blue(const std::string str);
                static void cyan(const std::string str);
        };

} // namespace robot_localization

#endif