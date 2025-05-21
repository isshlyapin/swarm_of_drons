#pragma once

#include <cstddef>
#include <list>
#include <rclcpp/logging.hpp>
#include <set>
#include <cassert>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

#include "navigator2/config.hpp"

enum class OptimalPathMode : std::uint8_t {
    MIN_DISTANCE = 0
};

class Vertex {
public:
    using Point = Eigen::Vector3d;

    Vertex(std::string vertexId, Point position)
        : id(std::move(vertexId)), position(std::move(position)) {}

    const std::string& getId() const { return id; }
    const Point& getPosition() const { return position; }

    void addNeighbor(const std::shared_ptr<Vertex>& neighbor, double timeOpen = 0.0) {
        neighbors[neighbor->getId()] = neighbor;
        lengths[neighbor->getId()]   = (neighbor->getPosition() - position).norm();
        timesOpen[neighbor->getId()] = timeOpen;
    }

    const std::unordered_map<std::string, std::shared_ptr<Vertex>>& getNeighbors() const {
        return neighbors;
    }

    bool isCanFlyAcross() const {
        return id[0] == 'p';
    }

    bool isHasNeighbors() const {
        return !neighbors.empty();
    }

    double getLength(const std::string& vertexId) const {
        assert(lengths.find(id) != lengths.end());
        return lengths.at(vertexId);
    }

    double getTimeOpen(const std::string& vertexId) const {
        assert(timesOpen.find(id) != timesOpen.end());
        return timesOpen.at(vertexId);
    }

    void setTimeOpen(const std::string& vertexId, double timeOpen) {
        assert(timesOpen.find(id) != timesOpen.end());
        timesOpen[vertexId] = timeOpen;
    }

private:
    std::string id;
    Point position;
    std::unordered_map<std::string, double> lengths;
    std::unordered_map<std::string, double> timesOpen;
    std::unordered_map<std::string, std::shared_ptr<Vertex>> neighbors;
};

struct Path {
    [[nodiscard]] bool isNotInPath(const std::string& vertexId) const {
        return visited.find(vertexId) == visited.end();
    }

    [[nodiscard]] bool isEmpty() const {
        return vertexes.empty();
    }

    [[nodiscard]] size_t size() const {
        return vertexes.size();
    }

    [[nodiscard]] double getDistance() const {
        if (size() < 2) {
            return 0.0;
        }
        
        double distance = 0.0;
        for (size_t i = 1; i < vertexes.size(); ++i) {
            distance += vertexes[i-1]->getLength(vertexes[i]->getId());
        }
        return distance;
    }

    [[nodiscard]] const std::shared_ptr<Vertex>& operator[](size_t index) const {
        assert(index < vertexes.size());
        return vertexes[index];
    }

    [[nodiscard]] const std::shared_ptr<Vertex>& back() const {
        assert(!vertexes.empty());
        return vertexes.back();
    }
    [[nodiscard]] const std::shared_ptr<Vertex>& front() const {
        assert(!vertexes.empty());
        return vertexes.front();
    }

    [[nodiscard]] bool operator==(const Path& other) const {
        return vertexes == other.vertexes;
    }

    void addVertex(const std::shared_ptr<Vertex>& vertex) {
        visited.insert(vertex->getId());
        vertexes.push_back(vertex);
    }

private:
    std::set<std::string> visited;
    std::vector<std::shared_ptr<Vertex>> vertexes;
};

struct Mission {
    double timeStart{};
    double timeFinish{};
    std::vector<double> velocities;
    std::vector<std::string> vertexes;
};

struct DroneVMax {
    double value;
};

struct DroneVMin {
    double value;
};

struct DroneFreeTime {
    double value;
};

struct DroneVOptimal {
    double value;
};

class Drone {
public:
    Drone(std::string vertex_id, DroneVMax vmax, DroneVMin vmin, DroneFreeTime free_time)
    : vertexId(std::move(vertex_id)), 
      vmax(vmax),
      vmin(vmin),
      freeTime(free_time)
    {
        initVoptimal();
    }

    Drone(Drone&&)                 = default;
    virtual ~Drone()               = default;
    Drone(const Drone&)            = default;
    Drone& operator=(Drone&&)      = default;
    Drone& operator=(const Drone&) = default;

    [[nodiscard]] double getVmax()     const { return vmax.value; }
    [[nodiscard]] double getVmin()     const { return vmin.value; }
    [[nodiscard]] double getVoptimal() const { return voptimal.value;  }

    [[nodiscard]] const std::string& getVertexId() const { return vertexId; }
    
    [[nodiscard]] double getFreeTime() const {
        return freeTime.value;
    }

    void setVmax(double new_vmax) {
        assert(new_vmax > vmin.value);
        vmax.value = new_vmax;
        initVoptimal();
    }

    void setVmin(double new_vmin) {
        assert(new_vmin > 0);
        vmin.value = new_vmin;
        initVoptimal();
    }

    void setFreeTime(double new_free_time) {
        assert(new_free_time > 0);
        freeTime.value = new_free_time;
    }

private:
    virtual void initVoptimal() {
        voptimal.value = (vmax.value + vmin.value) / 2;
    }

    std::string vertexId;
    
    DroneVMax vmax{};
    DroneVMin vmin{};
    DroneFreeTime freeTime{};
    DroneVOptimal voptimal{};
};

class Map {
public:
    using Point = Eigen::Vector3d;

    explicit Map(double safetyTime = 0.0)
        : safetyTime(safetyTime) {};

    size_t size() const {
        return vertexes.size();
    }

    static double getEpsilon() {
        return map_cfg::COORDINATE_TOLERANCE;
    }

    const Point& getVertexPosition(const std::string& vertexId) const {
        assert(vertexes.find(vertexId) != vertexes.end());
        return vertexes.at(vertexId)->getPosition();
    } 

    std::string getVertexId(const Point& position) const {
        for (const auto& [vertexId, vertex] : vertexes) {
            if ((vertex->getPosition() - position).norm() < getEpsilon()) {
                return vertexId;
            }
        }
        return std::string{""};
    }

    void addVertex(const std::string& vertexId, const Point& position) {
        auto vertex = std::make_shared<Vertex>(vertexId, position);
        vertexes[vertexId] = vertex;
    }

    void addEdge(const std::string& id_from, const std::string& id_to, double timeOpen = 0.0) {
        assert(vertexes.find(id_from) != vertexes.end());
        assert(vertexes.find(id_to) != vertexes.end());

        vertexes[id_from]->addNeighbor(vertexes[id_to], timeOpen);
    }

    Mission generateMission(const std::string& id_from, const std::string& id_to, 
                            const Drone& drone, OptimalPathMode mode) {
        assert(vertexes.find(id_from) != vertexes.end());
        assert(vertexes.find(id_to) != vertexes.end());

        std::list<Path> paths;
        generatePaths(id_from, id_to, paths);

        const Path optimalPath = findOptimalPath(paths, drone, mode);
        assert(!optimalPath.isEmpty());
        
        Mission mission;
        fieldMission(optimalPath, drone, mission);

        updateMap(mission);

        return mission;
    }

    void updateMap(const Mission& mission) {
        double timeOpen = mission.timeStart;
        for (size_t i = 0; i < mission.vertexes.size() - 1; ++i) {
            auto vertexId = mission.vertexes[i];
            auto nextVertexId = mission.vertexes[i+1];
            timeOpen += vertexes[vertexId]->getLength(nextVertexId) / mission.velocities[i];
            vertexes[vertexId]->setTimeOpen(nextVertexId, timeOpen);
        }
    }

    void fieldMission(const Path& path, const Drone& drone, Mission& mission) const {
        assert(!path.isEmpty());
       
        double timeStart = path[0]->getTimeOpen(path[1]->getId());

        double length = 0.0;

        for (size_t i = 1; i < path.size() - 1; ++i) {
            length += path[i-1]->getLength(path[i]->getId());
            const double timeFly = timeStart + (length / drone.getVoptimal());
            const double timeOpen = path[i]->getTimeOpen(path[i+1]->getId());
            if (timeFly < timeOpen + getSafetyTime()) {
                const double dtime = (timeOpen - timeFly) + getSafetyTime();
                timeStart += dtime;
            }
        }

        mission.timeStart  = timeStart;
        mission.timeFinish = timeStart + getSafetyTime() + (path.getDistance() / drone.getVoptimal());
        mission.vertexes.push_back(path[0]->getId());
        for (size_t i = 1; i < path.size(); ++i) {
            mission.velocities.push_back(drone.getVoptimal());
            mission.vertexes.push_back(path[i]->getId());
        }
    }

    double getSafetyTime() const {
        return safetyTime;
    }

    Path findOptimalPath(const std::list<Path>& paths, Drone drone, 
                         OptimalPathMode mode) const {
        assert(!paths.empty());

        if (mode == OptimalPathMode::MIN_DISTANCE) {
            return findMinimalDistancePath(paths);
        }
        
        return {};
    }

    static const Path& findMinimalDistancePath(const std::list<Path>& paths)  {
        assert(!paths.empty());
        return *std::min_element(paths.begin(), paths.end(),
            [](const Path& pathA, const Path& pathB) {
                return pathA.getDistance() < pathB.getDistance();
            });
    }

    void generatePaths(const std::string& id_from, const std::string& id_to, 
                       std::list<Path>& paths) {
        assert(vertexes.find(id_from) != vertexes.end());
        assert(vertexes.find(id_to) != vertexes.end());
        
        paths.emplace_front();
        auto curPathIt = paths.begin();
        recursiveGeneratePath(id_from, id_to, paths, curPathIt);
    }

private:
    void recursiveGeneratePath(const std::string& id_from, const std::string& id_to, 
                               std::list<Path>& paths, std::list<Path>::iterator& curPathIt) {
        assert(vertexes.find(id_from) != vertexes.end());
        assert(vertexes.find(id_to) != vertexes.end());

        if (id_from == id_to) {
            (*curPathIt).addVertex(vertexes[id_from]);
            return;
        }

        if (vertexes[id_from]->isHasNeighbors() &&
            (*curPathIt).isNotInPath(id_from)) 
        {
            (*curPathIt).addVertex(vertexes[id_from]);
            bool curPathUse = false;

            for (const auto& [vertexId, neighbor]: vertexes[id_from]->getNeighbors()) {
                if (!neighbor->isCanFlyAcross() && vertexId != id_to) {
                     continue; 
                }

                if (!curPathUse) {
                    curPathUse = true;
                    recursiveGeneratePath(neighbor->getId(), id_to, paths, curPathIt);
                } else {
                    paths.emplace_front(*curPathIt);
                    auto newPathIt = paths.begin();
                    recursiveGeneratePath(id_from, id_to, paths, newPathIt);
                }
            }
            return;
        }
    }

    double safetyTime;
    std::unordered_map<std::string, std::shared_ptr<Vertex>> vertexes;    
};