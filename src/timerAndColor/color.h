#pragma once

#include <iostream>
#include <string>

namespace myColor
{
    enum color
    {
        BLACK = 0,
        RED = 1,
        GREEN = 2,
        YELLOW = 3,
        BLUE = 4,
        PURPLE = 5,
        DGREEN = 6,
        WHITE = 7,
        DEFAULT = 8
    };
    template <bool bold = false, myColor::color C = myColor::DEFAULT>
    class begin
    {
    public:
        begin()
        {
            static const std::string color_table[8] = {"30m", "31m", "32m", "33m", "34m", "35m", "36m", "37m"};
            if (C != myColor::DEFAULT)
                if (bold)
                    std::cout << "\e[1;" + color_table[C];
                else
                    std::cout << "\e[" + color_table[C];
        }
        ~begin()
        {
            if (C != myColor::DEFAULT)
                std::cout << "\033[0m";
        }
    };

}

#define VARGS_(_10, _9, _8, _7, _6, _5, _4, _3, _2, _1, N, ...) N
#define VARGS(...) VARGS_(__VA_ARGS__, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)

#define CONCAT_(a, b) a##b
#define CONCAT(a, b) CONCAT_(a, b)

#define CREATE_INFO(C, B, ...)                                               \
    do                                                                       \
    {                                                                        \
        myColor::begin<B, myColor::C> CONCAT(lldonousethissuffix, __LINE__); \
        std::cout << __VA_ARGS__;                                            \
    } while (0)

#define BLACK_INFO(B, ...) CREATE_INFO(BLACK, B, __VA_ARGS__)
#define RED_INFO(B, ...) CREATE_INFO(RED, B, __VA_ARGS__)
#define GREEN_INFO(B, ...) CREATE_INFO(GREEN, B, __VA_ARGS__)
#define YELLOW_INFO(B, ...) CREATE_INFO(YELLOW, B, __VA_ARGS__)
#define BLUE_INFO(B, ...) CREATE_INFO(BLUE, B, __VA_ARGS__)
#define PURPLE_INFO(B, ...) CREATE_INFO(PURPLE, B, __VA_ARGS__)
#define DGREEN_INFO(B, ...) CREATE_INFO(DGREEN, B, __VA_ARGS__)
#define WHITE_INFO(B, ...) CREATE_INFO(WHITE, B, __VA_ARGS__)

#define DEFAULT_INFO(B, ...) CREATE_INFO(DEFAULT, B, __VA_ARGS__)
#define SHOW_VAR(S) BLUE_INFO(true, #S << " = " << S << std::endl)