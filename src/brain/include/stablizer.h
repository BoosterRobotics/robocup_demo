/**
 * @file stablizer.h
 * @brief 定义一个 Stablizer Class 用于防抖
 */
#pragma once

#include <vector>
#include <chrono>
#include <iterator>
#include <limits>

using namespace std;
namespace chr = std::chrono;
// Stablizer: 记录一定时间内数据, 进行处理, 防止抖动
class Stablizer
{
public:
    Stablizer()
    {
        _timespan = chr::milliseconds(2000);
    };

    void setTimespan(int msec)
    {
        _timespan = chr::milliseconds(msec);
    };

    // 加入新的数据
    void push(double x)
    {
        auto cur_time = chr::high_resolution_clock::now();
        _t.push_back(cur_time);
        _x.push_back(x);
        _removeOldData();
    };

    // 获取当前平滑后的数据, method: mean | max | min
    double getStablized(string method)
    {
        _removeOldData();
        if (_x.size() == 0)
            return 0;

        if (method == "mean")
        {
            double sum = 0;
            for (double num : _x)
            {
                sum += num;
            }
            return sum / _x.size();
        }

        if (method == "min")
        {
            double min = std::numeric_limits<double>::max();
            for (double num : _x)
            {
                if (num < min)
                {
                    min = num;
                }
            }
            return min;
        }

        if (method == "max")
        {
            double max = -std::numeric_limits<double>::max();
            for (double num : _x)
            {
                if (num > max)
                {
                    max = num;
                }
            }
            return max;
        }

        return 0;
    };

private:
    chr::duration<int, milli> _timespan;
    vector<double> _x;
    vector<chr::high_resolution_clock::time_point> _t;
    void _removeOldData()
    {
        auto curTime = chr::high_resolution_clock::now();
        if (_t.size() == 0)
            return;

        int i;
        for (i = 0; i < _t.size(); i++)
        {
            if (_t[i] + _timespan > curTime)
                break;
        }
        _t.erase(_t.begin(), std::next(_t.begin(), i));
        _x.erase(_x.begin(), std::next(_x.begin(), i));
    }
};