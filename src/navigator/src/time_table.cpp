#include "../include/time_table.hpp"
#include <cstddef>
#include <cstdio>
#include <cmath>
#include <rcl/time.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

rclcpp::Time TimeTable::infinity = rclcpp::Time(std::numeric_limits<int32_t>::max() - 10, std::numeric_limits<uint32_t>::max() - 1, RCL_ROS_TIME);

TimeTable::TimeTable() = default;

TimeTable::TimeTable(std::vector<std::pair<rclcpp::Time, rclcpp::Time>>& t) : times(t) {}

TimeTable::TimeTable(const TimeTable& other) {
    times = other.times;
}

TimeTable& TimeTable::operator+=(const TimeTable& other) {
    for (size_t i = 0; i < other.getSize(); i++) {
        AppendTime(other.getTime(i));
    }
    return *this;
}

void TimeTable::AppendTime(std::pair<rclcpp::Time, rclcpp::Time> time) {
    if (isFirstTimeBigger(time.first, time.second)) {
        RCLCPP_ERROR(rclcpp::get_logger("graph"), "time interval is incorrect in AppendTime");
        return;
    } else if (time.first.get_clock_type() != 1 || time.second.get_clock_type() != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("graph"), "Not sim time in CalcPossTimeEdgeAndNode");
        return;
    }

    size_t i1 = 0;
    while ( (i1 < times.size()) && (isFirstTimeBigger(time.first, times[i1].second)) ) {i1++;}
    size_t i2 = i1;
    while ( (i2 < times.size()) && (isFirstTimeBigger(time.second, times[i2].first)) ) {i2++;}

    if ((i2 < times.size()) && isTimesEqual(time.second, times[i2].first)) i2++;

    if (i1 == i2) times.insert(times.begin() + i1, time);
    else {
        if (isFirstTimeBigger(time.first, times[i1].first))  time.first  = times[i1].first;
        if (isFirstBiggerOrEq(times[i2 - 1].second, time.second)) time.second = times[i2 - 1].second;
        times.insert(times.begin() + i1, time);
        times.erase(times.begin() + i1 + 1, times.begin() + i2 + 1);
    }
}

void TimeTable::DeleteTime(const std::pair<rclcpp::Time, rclcpp::Time>& time) {
    if (isFirstTimeBigger(time.first, time.second)) {
        RCLCPP_ERROR(rclcpp::get_logger("my_logger"), "time interval is incorrect in DeleteTime");
        return;
    } else if (time.first.get_clock_type() != 1 || time.second.get_clock_type() != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("graph"), "Not sim time in CalcPossTimeEdgeAndNode");
        return;
    }

    size_t i1 = 0;
    while ( (i1 < times.size()) && (isFirstTimeBigger(time.first, times[i1].second)) ) i1++;
    size_t i2 = i1;
    while ( (i2 < times.size()) && (isFirstTimeBigger(time.second, times[i2].first)) ) i2++;

    if (i2 - 1 == i1) {
        if (isFirstTimeBigger(time.first, times[i1].first) &&
            isFirstTimeBigger(times[i1].second, time.second)) {
            std::pair<rclcpp::Time, rclcpp::Time> t1 = {times[i1].first, time.first};
            times.insert(times.begin() + i1, t1);
            times[i1 + 1].first = time.second;
        } else if ( isFirstTimeBigger(time.first, times[i1].first) &&
                    isFirstBiggerOrEq(time.second, times[i1].second)) {
            times[i1].second = time.first;
        } else if ( isFirstTimeBigger(time.second, times[i1].first) &&
                    isFirstTimeBigger(times[i1].second, time.second)) {
            times[i1].first = time.second;
        } else {
            times.erase(times.begin() + i1);
        }
    }
    else if (i2 > 0 && i2 - 1 > i1) {
        if ( isFirstTimeBigger(time.first, times[i1].first) &&
             isFirstTimeBigger(times[i1].second, time.first)) {
            times[i1].second = time.first;
            i1++;
        }
        if (isFirstTimeBigger(times[i2 - 1].second, time.second) &&
            isFirstTimeBigger(time.second, times[i2 - 1].first)) {
            times[i2 - 1].first = time.second;
            i2--;
        }
        if (i2 >= i1) {
            times.erase(times.begin() + i1, times.begin() + i2);
        }
    }
}

const std::vector<std::pair<rclcpp::Time, rclcpp::Time>>& TimeTable::getTimes() const {
    return times;
}

TimeTable& TimeTable::operator-=(const TimeTable& other) {
    for (size_t i = 0; i < other.getSize(); i++) {
        DeleteTime(other.getTime(i));
    }
    return *this;
}

TimeTable& TimeTable::operator=(TimeTable& other) {
    times = other.getTimes();
    return *this;
}

bool TimeTable::operator==(const TimeTable& other) const {
    if (times.size() != other.getSize()) return false;

    for (size_t i = 0; i < times.size(); i++) {
        if (!isTimesEqual(times[i].first,  other.getTime(i).first) ||
            !isTimesEqual(times[i].second, other.getTime(i).second)) {
                return false;
            }
    }
    return true;
}

bool TimeTable::operator!=(const TimeTable& other) const {
    return !(other == *this);
}

void TimeTable::ClearTimes() {times.clear();}

size_t TimeTable::getSize() const {return times.size();}

const std::pair<rclcpp::Time, rclcpp::Time>& TimeTable::getTime(size_t index) const {
    if (index >= times.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("timetable"), "Wrong index in getTime");
    }
    return times[index];
}

rclcpp::Time TimeTable::TimeFloatToRCL(double time) {
    int inttime = time;
    if (time < 0) return infinity;
    return rclcpp::Time( inttime * 1e9 + (int)( (time - inttime) * 1e9), RCL_ROS_TIME );
}

bool TimeTable::isInfinity(rclcpp::Time& time) {
    double eps = 1;
    if (time.seconds() + eps >= infinity.seconds()) return true;
    return false;
}

bool TimeTable::isTimeInTimes(std::pair<rclcpp::Time, rclcpp::Time>& time) {
    for (size_t i = 0; i < times.size(); i++) {
        if ( isFirstTimeBigger(time.first, times[i].first) && 
             isFirstTimeBigger(times[i].second, time.second)) {
            return true;
        }
    }
    return false;
}

void TimeTable::PrintTimes() const {
    RCLCPP_INFO(rclcpp::get_logger("graph"), "Start printing times:");
    for (size_t i = 0; i < times.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("my_logger"), "[%f, %f]",
            times[i].first.seconds(), times[i].second.seconds());
    }
    RCLCPP_INFO(rclcpp::get_logger("graph"), "End printing times");
}

bool TimeTable::isFirstTimeBigger(const rclcpp::Time& time1, const rclcpp::Time& time2) {
    double eps = 1e-3;
    return (time1.seconds() >= time2.seconds() + eps);
}

bool TimeTable::isTimesEqual(const rclcpp::Time& time1, const rclcpp::Time& time2) {
    double eps = 1e-3;
    return (   (time1.seconds() - eps) <= time2.seconds()
            && time2.seconds() <= (time1.seconds() + eps));
}

bool TimeTable::isFirstBiggerOrEq(const rclcpp::Time& time1, const rclcpp::Time& time2) {
    return (isFirstTimeBigger(time1, time2) || isTimesEqual(time1, time2));
}

bool TimeTable::isTimeIn(rclcpp::Time time) {
    for (size_t i = 0; i < times.size(); i++) {
        if ( isFirstTimeBigger(time, times[i].first) && 
                isFirstTimeBigger(times[i].second, time)) {
            return true;
        }
    }
    return false;
}

bool TimeTable::checkCollisions(std::pair<rclcpp::Time, rclcpp::Time> time) {
    TimeTable news = *this;
    news.DeleteTime(time);
    if (this->getTimes() == news.getTimes()) return false;
    return true;
}