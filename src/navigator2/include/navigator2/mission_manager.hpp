#pragma once 

#include <queue>
#include <string>
#include <vector>

struct InputMission {
    std::string id_from;
    std::string id_to;
    double time_appearance;
};

struct InputMissionCompare {
    bool operator()(const InputMission& lhs, const InputMission& rhs) const {
        return lhs.time_appearance > rhs.time_appearance;
    }
};


class MissionManager {
public:
    MissionManager() = default;

    [[nodiscard]] bool empty() const;
    
    [[nodiscard]] size_t size() const;

    [[nodiscard]] const InputMission& topInputMission() const;
    
    void popInputMission();

    void loadMissionsFromCSV(const std::string& file_path);

    void addMission(const std::string& id_from, const std::string& id_to, 
        double time_appearance);

private:
    std::priority_queue<InputMission, std::vector<InputMission>, InputMissionCompare> mission_queue_;
};