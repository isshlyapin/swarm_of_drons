#pragma once

#include <vector>
#include <utility>
#include <rclcpp/rclcpp.hpp>

class TimeTable {
private:
    std::vector<std::pair<rclcpp::Time, rclcpp::Time>> times;
    static rclcpp::Time infinity;

public:
    TimeTable();
    TimeTable(std::vector<std::pair<rclcpp::Time, rclcpp::Time>>& t);
    TimeTable(const TimeTable& other);

    void AppendTime(std::pair<rclcpp::Time, rclcpp::Time> time);
    void DeleteTime(const std::pair<rclcpp::Time, rclcpp::Time>& time);
    
    TimeTable& operator+=(const TimeTable& other);
    TimeTable& operator-=(const TimeTable& other);
    TimeTable& operator=(TimeTable& other);
    bool operator==(const TimeTable& other) const;
    bool operator!=(const TimeTable& other) const;

    void ClearTimes();
    size_t getSize() const;
    const std::pair<rclcpp::Time, rclcpp::Time>& getTime(size_t index) const;

    static rclcpp::Time TimeFloatToRCL(double time);
    static bool isInfinity(rclcpp::Time& time);
    static bool isFirstTimeBigger(const rclcpp::Time& time1, const rclcpp::Time& time2);
    static bool isTimesEqual(const rclcpp::Time& time1, const rclcpp::Time& time2);
    static bool isFirstBiggerOrEq(const rclcpp::Time& time1, const rclcpp::Time& time2);
    bool isTimeInTimes(std::pair<rclcpp::Time, rclcpp::Time>& time);

    const std::vector<std::pair<rclcpp::Time, rclcpp::Time>>& getTimes() const;

    void PrintTimes() const;
};
