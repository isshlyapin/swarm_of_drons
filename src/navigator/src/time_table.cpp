#include "../include/time_table.hpp"
#include <cstddef>
#include <cstdio>
#include <cmath>

rclcpp::Time TimeTable::infinity = rclcpp::Time(std::numeric_limits<int32_t>::max() - 1, std::numeric_limits<uint32_t>::max() - 1);

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
    if (time.second < time.first) {
        RCLCPP_ERROR(rclcpp::get_logger("my_logger"), "time interval is incorrect in AppendTime");
        return;
    }

    size_t i1 = 0;
    while ( (i1 < times.size()) && (time.first  > times[i1].second) ) i1++;
    size_t i2 = i1;
    while ( (i2 < times.size()) && (time.second >= times[i2].first) ) i2++;
    // printf("i1: %ld\n", i1);
    // printf("i2: %ld\n", i2);
    // printf("first time:  %f\n", time.first.seconds());
    // printf("second time: %f\n", time.second.seconds());
    if (i1 == i2) times.insert(times.begin() + i1, time);
    else {
        if (time.first  > times[i1].first)  time.first  = times[i1].first;
        if (i2 < times.size() && time.second < times[i2].second) time.second = times[i2].second;
        times.insert(times.begin() + i1, time);
        times.erase(times.begin() + i1 + 1, times.begin() + i2 + 1);
    }
    // printf("In times first:  %f\n", times[0].first.seconds());
    // printf("In times second: %f\n", times[0].second.seconds());
}

void TimeTable::DeleteTime(const std::pair<rclcpp::Time, rclcpp::Time>& time) {
    if (time.second < time.first) {
        RCLCPP_ERROR(rclcpp::get_logger("my_logger"), "time interval is incorrect in DeleteTime");
        return;
    }

    size_t i1 = 0;
    while ( (i1 < times.size()) && (time.first  > times[i1].second) ) i1++;
    size_t i2 = i1;
    while ( (i2 < times.size()) && (time.second >= times[i2].first) ) i2++;

    if (i2 - 1 == i1) {
        if (time.first > times[i1].first && time.second < times[i1].second) {
            std::pair<rclcpp::Time, rclcpp::Time> t1 = {times[i1].first, time.first};
            times.insert(times.begin() + i1, t1);
            times[i1 + 1].first = time.second;
        } else if (time.first > times[i1].first && time.second >= times[i1].second) {
            times[i1].second = time.first;
        } else if (time.second < times[i1].second) {
            times[i1].first = time.second;
        } else {
            times.erase(times.begin() + i1);
        }
    }
    else if (i2 - 1 > i1) {
        if (time.first > times[i1].first) {
            times[i1].second = time.first;
            i1++;
        }
        if ( (i2 < times.size()) &&
            (time.second < times[i2 - 1].second && time.second != times[i2 - 1].first) ) {
            times[i2 - 1].second = time.second;
            i2--;
        }
        if (i2 >= i1) {
            times.erase(times.begin() + i1, times.begin() + i2 + 1);
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
    if (other.getTimes() == times) return true;
    return false;
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

void TimeTable::editTime(size_t index, rclcpp::Time& t1, rclcpp::Time& t2) {
    if (index >= times.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("timetable"), "Wrong index in editTime");
    }
    DeleteTime(times[index]);
    std::pair<rclcpp::Time, rclcpp::Time> time = {t1, t2};
    AppendTime(time);
}

rclcpp::Time TimeTable::TimeFloatToRCL(float time) {
    int inttime = time;
    if (time < 0) return infinity;
    return rclcpp::Time( inttime * 1e9 + (int)( (time - inttime) * 1e9) );
}

bool TimeTable::isInfinity(rclcpp::Time& time) {
    if (time == infinity) return true;
    return false;
}

bool TimeTable::isTimeInTimes(std::pair<rclcpp::Time, rclcpp::Time>& time) {
    for (size_t i = 0; i < times.size(); i++) {
        if (time.first >= times[i].first && time.second <= times[i].second) {
            return true;
        }
    }
    return false;
}

void TimeTable::PrintTimes() const {
    for (size_t i = 0; i < times.size(); ++i) {
        RCLCPP_INFO(rclcpp::get_logger("my_logger"), "[%f, %f]",
            times[i].first.seconds(), times[i].second.seconds());
    }
}
