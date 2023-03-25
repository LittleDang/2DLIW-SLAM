#pragma once
#include "timerAndColor/timer.h"
#include "utilies/params.h"
#include <fstream>
#include <map>
#include <stack>
#include <type_traits>
#include <vector>
namespace lvio_2d
{

    // 只可以运行在单线程下
    class record
    {
    public:
        record(const std::string &filename) : filename(filename){};
        record(){};
        void set_filename(const std::string &filename_) { filename = filename_; }
        ~record()
        {
            if (!PARAM(output_tum))
                return;
            std::ofstream of;
            std::string path = PARAM(output_dir) + filename + ".md";
            of.open(path, std::ios_base::out);
            std::cout << "open " << path << ":" << of.is_open() << std::endl;
            if (!of.is_open())
                return;

            of << "time_recorder" << std::endl;
            of << "size of total record type:" << time_recorder.size() << std::endl
               << std::endl;
            of << "| type name | record size | max(us) | min(us) | aver(us) | variance(${us}^2$) |" << std::endl
               << "| --- | --- | --- | --- | --- | --- |" << std::endl;
            for (auto &[type_name, records] : time_recorder)
            {
                of << "| " << type_name << " | " << records.size() << " | ";
                uint64_t total = 0;
                uint64_t max = 0;
                uint64_t min = 999999999;
                for (auto item : records)
                {
                    total += item;
                    if (item > max)
                        max = item;
                    if (item < min)
                        min = item;
                }
                double aver = 0;
                double variance = 0;
                if (records.size() > 0)
                    aver = double(total) / records.size();
                for (auto item : records)
                {
                    variance += (item - aver) * (item - aver);
                }
                variance /= records.size();
                of << max << " |" << min << " |" << aver << " |" << variance << " |" << std::endl;
            }

            of << std::endl;
            of << "others_recorder" << std::endl;
            of << "size of total record type:" << others_recorder.size() << std::endl
               << std::endl;
            of << "| type name | record size | max | min | aver | variance |" << std::endl
               << "| --- | --- | --- | --- | --- | --- |" << std::endl;
            for (auto &[type_name, records] : others_recorder)
            {
                of << "| " << type_name << " | " << records.size() << " | ";
                uint64_t total = 0;
                uint64_t max = 0;
                uint64_t min = 999999999;
                for (auto item : records)
                {
                    total += item;
                    if (item > max)
                        max = item;
                    if (item < min)
                        min = item;
                }
                double aver = 0;
                double variance = 0;
                if (records.size() > 0)
                    aver = double(total) / records.size();
                for (auto item : records)
                {
                    variance += (item - aver) * (item - aver);
                }
                variance /= records.size();
                of << max << " |" << min << " |" << aver << " |" << variance << " |" << std::endl;
            }
            of.close();
        };
        // void clear_all();
        // void clear_sometye(const std::string &type_name);
        void begin_record()
        {
            micro_timers.emplace(new myTimer::microTimer<>::type);
            micro_timers.top()->end("", true, false);
        }
        void end_record(const std::string &type_name)
        {

            uint64_t dt = micro_timers.top()->end("", true, false);
            micro_timers.pop();
            auto iter = time_recorder.find(type_name);
            if (iter == time_recorder.end())
                time_recorder[type_name] = {dt};
            else
                iter->second.push_back(dt);
        }
        void add_record(const std::string &type_name, uint64_t v)
        {

            auto iter = others_recorder.find(type_name);
            if (iter == others_recorder.end())
                others_recorder[type_name] = {v};
            else
                iter->second.push_back(v);
        }

    private:
        // myTimer::microTimer<>::type mT;
        std::stack<std::unique_ptr<myTimer::microTimer<>::type>> micro_timers;
        std::map<std::string, std::vector<uint64_t>> time_recorder;
        std::map<std::string, std::vector<uint64_t>> others_recorder;
        std::string filename;
    };
} // namespace lvio_2d