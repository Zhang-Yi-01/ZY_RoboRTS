/*
 * @Description: 终端彩色输出
 * @Author: ZY
 * @Date: 2022.10.24
 */
#ifndef COLOR_TERMINAL_HPP
#define COLOR_TERMINAL_HPP

// c++
#include <string>

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

                static void green(const std::string str);
                static void red(const std::string str);
                static void yellow(const std::string str);
                static void white(const std::string str);
                static void blue(const std::string str);
                static void cyan(const std::string str);
        };

} // namespace robot_localization

#endif