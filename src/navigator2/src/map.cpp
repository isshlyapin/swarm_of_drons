#include <fastcsv/csv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "navigator2/map.hpp"
#include "navigator2/config.hpp"

std::string Map::getVertexId(const Point& position) const {
    for (const auto& [vertexId, vertex] : vertexes) {
        if ((vertex->getPosition() - position).norm() < getEpsilon()) {
            return vertexId;
        }
    }
    return std::string{""};
}

void Map::addVertex(const std::string& vertexId, const Point& position) {
    auto vertex = std::make_shared<Vertex>(vertexId, position);
    vertexes[vertexId] = vertex;
}

void Map::addEdge(const std::string& id_from, const std::string& id_to, 
                  double timeOpen) {
    vertexes[id_from]->addNeighbor(vertexes[id_to], timeOpen);
}

Mission Map::generateMission(const std::string& id_from, const std::string& id_to, 
    const Drone& drone, map_cfg::OptimalPathMode mode) {
    std::list<Path> paths;
    generatePaths(id_from, id_to, paths);

    const Path optimalPath = findOptimalPath(paths, drone, mode);

    Mission mission;
    fieldMission(optimalPath, drone, mission);

    updateMap(mission);

    return mission;
}

void Map::updateMap(const Mission& mission) {
    double timeOpen = mission.timeStart;
    for (size_t i = 0; i < mission.vertexes.size() - 1; ++i) {
        auto vertexId = mission.vertexes[i];
        auto nextVertexId = mission.vertexes[i+1];
        timeOpen += vertexes[vertexId]->getLength(nextVertexId) / mission.velocities[i];
        vertexes[vertexId]->setTimeOpen(nextVertexId, timeOpen + getSafetyTime());
    }
}

void Map::fieldMission(const Path& path, const Drone& drone, Mission& mission) const {
    assert(!path.isEmpty());
   
    double timeStart = path[0]->getTimeOpen(path[1]->getId());

    double length = 0.0;

    for (size_t i = 1; i < path.size() - 1; ++i) {
        length += path[i-1]->getLength(path[i]->getId());
        const double timeFly = timeStart + (length / drone.getVoptimal());
        const double timeOpen = path[i]->getTimeOpen(path[i+1]->getId());
        if (timeFly < timeOpen) {
            const double dtime = (timeOpen - timeFly);
            timeStart += dtime;
        }
    }

    mission.timeStart  = std::max(timeStart, drone.getFreeTime());
    mission.timeFinish = mission.timeStart + (path.getDistance() / drone.getVoptimal()) + getSafetyTime();
    mission.vertexes.push_back(path[0]->getId());
    for (size_t i = 1; i < path.size(); ++i) {
        mission.velocities.push_back(drone.getVoptimal());
        mission.vertexes.push_back(path[i]->getId());
    }
}

Path Map::findOptimalPath(const std::list<Path>& paths, const Drone& drone, 
    map_cfg::OptimalPathMode mode) {
    if (mode == map_cfg::OptimalPathMode::MIN_DISTANCE) {
        return findMinimalDistancePath(paths);
    } else if (mode == map_cfg::OptimalPathMode::MIN_TIME) {
        return findMinimalTimePath(paths, drone);
    }

    return {};
}

const Path& Map::findMinimalTimePath(const std::list<Path>& paths, const Drone& drone) {
    double resTimeFinish = std::numeric_limits<double>::max();
    const Path* resPath = &paths.front();
    for (const auto& path : paths) {
        double timeStart = path[0]->getTimeOpen(path[1]->getId());

        double length = 0.0;

        for (size_t i = 1; i < path.size() - 1; ++i) {
            length += path[i-1]->getLength(path[i]->getId());
            const double timeFly = timeStart + (length / drone.getVoptimal());
            const double timeOpen = path[i]->getTimeOpen(path[i+1]->getId());
            if (timeFly < timeOpen) {
                const double dtime = (timeOpen - timeFly);
                timeStart += dtime;
            }
        }
        timeStart  = std::max(timeStart, drone.getFreeTime());
        const double timeFinish = timeStart + (path.getDistance() / drone.getVoptimal());
        if (timeFinish < resTimeFinish) {
            resTimeFinish = timeFinish;
            resPath = &path;
        }
    }

    return *resPath;
}

const Path& Map::findMinimalDistancePath(const std::list<Path>& paths)  {
    return *std::min_element(paths.begin(), paths.end(),
        [](const Path& pathA, const Path& pathB) {
            return pathA.getDistance() < pathB.getDistance();
        });
}

void Map::generatePaths(const std::string& id_from, const std::string& id_to, 
                        std::list<Path>& paths) {
    paths.emplace_front();
    auto curPathIt = paths.begin();
    recursiveGeneratePath(id_from, id_to, paths, curPathIt);
}

void Map::recursiveGeneratePath(const std::string& id_from, 
                                const std::string& id_to, 
                                std::list<Path>& paths, 
                                std::list<Path>::iterator& curPathIt) {
    if (id_from == id_to) {
        (*curPathIt).addVertex(vertexes[id_from]);
        return;
    }

    if (vertexes[id_from]->isHasNeighbors() &&
        (*curPathIt).isNotInPath(id_from)) 
    {
        (*curPathIt).addVertex(vertexes[id_from]);

        for (const auto& [vertexId, neighbor]: vertexes[id_from]->getNeighbors()) {
            if (!neighbor->isCanFlyAcross() && vertexId != id_to) {
                continue; 
            }

            paths.emplace_front(*curPathIt);
            auto newPathIt = paths.begin();
            recursiveGeneratePath(neighbor->getId(), id_to, paths, newPathIt);
        }
    }

    paths.erase(curPathIt);
}

void Map::loadGraphFromCSV(const std::string& filePath) {
    io::CSVReader<4> in_csv(filePath);
    in_csv.read_header(io::ignore_extra_column, "index", "x", "y", "z");

    std::string index;
    double x = 0;
    double y = 0;
    double z = 0;

    while (in_csv.read_row(index, x, y, z)) {
        addVertex(index, {x, y, z});
    }
}

void Map::loadEdgesFromCSV(const std::string& filePath) {
    RCLCPP_INFO(rclcpp::get_logger("Map"), "Loading edges from %s", filePath.c_str());
    io::CSVReader<3> in_csv(filePath);
    in_csv.read_header(io::ignore_extra_column, "index1", "index2", "time_open");

    RCLCPP_INFO(rclcpp::get_logger("Map"), "Reading CSV file...");
    std::string id_from;
    std::string id_to;
    double time_open = 0;

    while (in_csv.read_row(id_from, id_to, time_open)) {
        RCLCPP_INFO(rclcpp::get_logger("Map"), "Adding edge: %s -> %s with time_open %lf", 
            id_from.c_str(), id_to.c_str(), time_open);
        addEdge(id_from, id_to, time_open);
    }
}
