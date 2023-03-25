#pragma once
#include "color.h"
#include <chrono>
#include <mutex>
#include <type_traits>

namespace myTimer
{
    /**
     * @brief 这是一个单例锁，主要是多个线程的时候多个模板的cout会乱，所以公用一个
     * 如果自己有想要输出的东西的时候，也可以用这个锁，防止和Timer的输出乱了。
     *
     */
    class onlyMutex
    {
    public:
        std::mutex &getMutex()
        {
            return Lock;
        }

    private:
        inline static std::mutex Lock;
    };

    /**
     * @brief 通用计时器，离开作用域自动释放，也可提前释放
     *
     * @tparam T std::chrono::duration<>,计时器类型
     * @tparam C myColor::color 颜色，缺省为黄色
     * @tparam B bold，缺省为true
     */
    template <typename T, myColor::color C = myColor::color::YELLOW, bool B = true>
    class Timer
    {
    public:
        /**
         * @brief Construct a new Timer object
         *
         * @param str 备注信息
         */
        Timer(const std::string &str = "") : str1(str), release(false),
                                             beginTime(std::chrono::system_clock::now())
        {
            // std::lock_guard<std::mutex> lk(oM.getMutex());
            // std::cout << &oM.getMutex() << std::endl;
        }

        /**
         * @brief 统计从创建或这上一次刷新到目前所过的时间
         *
         * @param str 输出的备注信息2
         * @param refresh 是否刷新起始时间，默认为false
         */
        uint64_t end(const std::string &str = "", bool refresh = false, bool output = true)
        {
            std::string timeType;
            if (std::is_same<T, std::chrono::microseconds>::value)
                timeType = "us";
            else if (std::is_same<T, std::chrono::milliseconds>::value)
                timeType = "ms";
            else if (std::is_same<T, std::chrono::seconds>::value)
                timeType = "s";

            auto end = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<T>(
                end - beginTime);
            if (refresh)
                beginTime = end;
            if (output)
            {
                std::lock_guard<std::mutex> lk(oM.getMutex());
                {
                    myColor::begin<B, myColor::color::YELLOW> b;
                    std::cout << str1 << str << "\tcost:" << duration.count() << timeType << std::endl;
                }
            }
            release = true;
            return duration.count();
        }
        ~Timer()
        {
            if (!release)
                end();
        }

    private:
        bool release;
        std::string str1;
        std::chrono::_V2::system_clock::time_point beginTime;
        onlyMutex oM;
    };

    /**
     * @brief microTimer,由于c++好像没办法typedef部分模板参数，所以只能再包装一层了
     *
     * @tparam C myColor::color 颜色，缺省为myColor::YELLOW
     * @tparam B bold ,缺省为true
     */
    template <myColor::color C = myColor::YELLOW, bool B = true>
    struct microTimer
    {
        typedef Timer<std::chrono::microseconds, C, B> type;
    };

    /**
     * @brief millTimer,由于c++好像没办法typedef部分模板参数，所以只能再包装一层了
     *
     * @tparam C myColor::color 颜色，缺省为myColor::YELLOW
     * @tparam B bold ,缺省为true
     */

    template <myColor::color C = myColor::color::YELLOW, bool B = true>
    struct millTimer
    {
        typedef Timer<std::chrono::milliseconds, C, B> type;
    };

    /**
     * @brief secTimer,由于c++好像没办法typedef部分模板参数，所以只能再包装一层了
     *
     * @tparam C myColor::color 颜色，缺省为myColor::YELLOW
     * @tparam B bold ,缺省为true
     */
    template <myColor::color C = myColor::YELLOW, bool B = true>
    struct secTimer
    {
        typedef Timer<std::chrono::seconds, C, B> type;
    };

} // namespace myTimer

#define doTimer_1(T, STR) myTimer::T<myColor::YELLOW, true>::type \
CONCAT(donusethissuffix, __LINE__)(STR)
#define doTimer_2(T, STR, C) myTimer::T<C, true>::type \
CONCAT(donusethissuffix, __LINE__)(STR)
#define doTimer_3(T, STR, C, B) myTimer::T<C, B>::type \
CONCAT(donusethissuffix, __LINE__)(STR)

/**
 * @brief 匿名microTimer,一般用这个宏的时候，表示你不在意它的名字，离开作用域自动销毁
 * 支持1-3个参数，分别是STR,COLOR,BOLD
 *
 */
#define lmicroTimer(...)                 \
    CONCAT(doTimer_, VARGS(__VA_ARGS__)) \
    (microTimer, __VA_ARGS__)

/**
 * @brief 匿名millTimer,一般用这个宏的时候，表示你不在意它的名字，离开作用域自动销毁
 * 支持1-3个参数，分别是STR,COLOR,BOLD
 *
 */
#define lmillTimer(...)                  \
    CONCAT(doTimer_, VARGS(__VA_ARGS__)) \
    (millTimer, __VA_ARGS__)

/**
 * @brief 匿名secTimer,一般用这个宏的时候，表示你不在意它的名字，离开作用域自动销毁
 * 支持1-3个参数，分别是STR,COLOR,BOLD
 *
 */
#define lsecTimer(...)                   \
    CONCAT(doTimer_, VARGS(__VA_ARGS__)) \
    (secTimer, __VA_ARGS__)
