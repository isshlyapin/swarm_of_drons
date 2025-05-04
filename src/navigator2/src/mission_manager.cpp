#include "navigator2/mission_manager.hpp"

#include <fastcsv/csv.hpp>

bool MissionManager::empty() const {
    return mission_queue_.empty();
}

size_t MissionManager::size() const {
    return mission_queue_.size();
}

const InputMission& MissionManager::topInputMission() const {
    return mission_queue_.top();
}

void MissionManager::popInputMission() {
    mission_queue_.pop();
}

void MissionManager::loadMissionsFromCSV(const std::string& file_path) {
    io::CSVReader<3> in_csv(file_path);
    in_csv.read_header(io::ignore_extra_column, "index1", "index2", "time_appearance");

    std::string id_from;
    std::string id_to;
    double time_appearance = 0.0;
    while (in_csv.read_row(id_from, id_to, time_appearance)) {
        addMission(id_from, id_to, time_appearance);
    }
}

void MissionManager::addMission(const std::string& id_from, const std::string& id_to, 
    double time_appearance) {
    const InputMission mission{id_from, id_to, time_appearance};
    mission_queue_.push(mission);
}
