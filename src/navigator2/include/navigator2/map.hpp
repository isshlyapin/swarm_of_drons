#pragma once

#include <cstddef>
#include <list>
#include <rclcpp/logging.hpp>
#include <set>
#include <cassert>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

enum class OptimalPathMode {
    MIN_DISTANCE = 0
};

class Vertex {
public:
    using Point = Eigen::Vector3d;

    Vertex(const std::string& id, const Point& position)
        : id(id), position(position) {}


    const std::string& getId() const { return id; }
    const Point& getPosition() const { return position; }

    void addNeighbor(const std::shared_ptr<Vertex>& neighbor, double timeOpen = 0.0) {
        neighbors.push_back(neighbor);
        lengths[neighbor->getId()] = (neighbor->getPosition() - position).norm();
        timesOpen[neighbor->getId()] = timeOpen;
    }

    const std::vector<std::shared_ptr<Vertex>>& getNeighbors() const {
        return neighbors;
    }

    bool isCanFlyAcross() const {
        return id[0] == 'p';
    }

    bool isHasNeighbors() const {
        return !neighbors.empty();
    }

    double getLength(const std::string& id) const {
        assert(lengths.find(id) != lengths.end());
        return lengths.at(id);
    }

    double getTimeOpen(const std::string& id) const {
        RCLCPP_INFO(rclcpp::get_logger("Vertex"), "getTimeOpen from: %s, to: %s", this->id.c_str(), id.c_str());
        assert(timesOpen.find(id) != timesOpen.end());
        return timesOpen.at(id);
    }

    void setTimeOpen(const std::string& id, double timeOpen) {
        assert(timesOpen.find(id) != timesOpen.end());
        timesOpen[id] = timeOpen;
    }

private:
    std::string id;
    Point position;
    std::vector<std::shared_ptr<Vertex>> neighbors;
    std::unordered_map<std::string, double> lengths;
    std::unordered_map<std::string, double> timesOpen;
};

struct Path {
    bool isNotInPath(const std::string& id) const {
        return visited.find(id) == visited.end();
    }

    bool isEmpty() const {
        return vertexes.empty();
    }

    size_t size() const {
        return vertexes.size();
    }

    double getDistance() const {
        double distance = 0.0;
        for (size_t i = 1; i < vertexes.size(); ++i) {
            distance += vertexes[i-1]->getLength(vertexes[i]->getId());
        }
        return distance;
    }

    void addVertex(const std::shared_ptr<Vertex>& vertex) {
        visited.insert(vertex->getId());
        vertexes.push_back(vertex);
    }

    std::set<std::string> visited;
    std::vector<std::shared_ptr<Vertex>> vertexes;
};

struct Mission {
    double timeStart;
    double timeFinish;
    std::vector<double> velocities;
    std::vector<std::string> vertexes;
};

class Drone {
public:
    Drone(const std::string& vertex_id, double vmax, double vmin, double free_time)
    : vmax(vmax), vmin(vmin), freeTime(free_time) , vertexId(vertex_id) {
        assert(vmax > vmin);
        assert(vmin > 0);

        initVoptimal();
    }

    double getVmax() const { return vmax; }
    double getVmin() const { return vmin; }
    double getVoptimal() const { return voptimal; }

    const std::string& getVertexId() const { return vertexId; }
    void setVmax(double new_vmax) {
        assert(new_vmax > vmin);
        vmax = new_vmax;
        initVoptimal();
    }

    void setVmin(double new_vmin) {
        assert(new_vmin > 0);
        vmin = new_vmin;
        initVoptimal();
    }

    virtual void initVoptimal() {
        voptimal = (vmax + vmin) / 2;
    }

    double getFreeTime() const {
        return freeTime;
    }

    void setFreeTime(double new_free_time) {
        freeTime = new_free_time;
    }

private:
    double vmax;
    double vmin;
    double voptimal;
    double freeTime;

    std::string vertexId;
};


class Map {
public:
    using Point = Eigen::Vector3d;

public:
    Map() = default;

    size_t size() const {
        return vertexes.size();
    }

    double getEpsilon() const {
        return 1.0; // Placeholder for epsilon value
    }

    const Point& getVertexPosition(const std::string& id) const {
        RCLCPP_INFO(rclcpp::get_logger("Map"), "getVertexPosition: %s", id.c_str());
        assert(vertexes.find(id) != vertexes.end());
        return vertexes.at(id)->getPosition();
    } 

    std::string getVertexId(const Point& position) const {
        for (const auto& [vertexId, vertex] : vertexes) {
            if ((vertex->getPosition() - position).norm() < getEpsilon()) {
                return vertexId;
            }
        }
        return std::string{""};
    }

    void addVertex(const std::string& id, const Point& position) {
        auto vertex = std::make_shared<Vertex>(id, position);
        vertexes[id] = vertex;
    }

    void addEdge(const std::string& id_from, const std::string& id_to, double timeOpen = 0.0) {
        assert(vertexes.find(id_from) != vertexes.end());
        assert(vertexes.find(id_to) != vertexes.end());

        vertexes[id_from]->addNeighbor(vertexes[id_to], timeOpen);
    }

    bool isPathsValid(const std::string& id_from, const std::string& id_to, const std::list<Path>& paths) {
        assert(vertexes.find(id_from) != vertexes.end());
        assert(vertexes.find(id_to) != vertexes.end());

        bool res = true;

        for (const auto& path: paths) {
            if (path.vertexes.empty()) {
                res = false;
                break;
            }

            if (path.vertexes.front()->getId() != id_from ||
                path.vertexes.back()->getId() != id_to) {
                res = false;
                break;
            }
        }

        return res;
    }

    Mission generateMission(const std::string& id_from, const std::string& id_to, 
                            Drone drone, OptimalPathMode mode) {
        assert(vertexes.find(id_from) != vertexes.end());
        assert(vertexes.find(id_to) != vertexes.end());

        std::list<Path> paths;
        generatePaths(id_from, id_to, paths);
        assert(isPathsValid(id_from, id_to, paths));

        Path optimalPath = findOptimalPath(paths, drone, mode);
        assert(!optimalPath.isEmpty());
        
        Mission mission;
        fieldMission(optimalPath, drone, mission);

        if (mission.timeStart < drone.getFreeTime()) {
            mission.timeStart = drone.getFreeTime();
        }

        RCLCPP_INFO(rclcpp::get_logger("Map"), "Start updateMap");
        updateMap(mission);
        RCLCPP_INFO(rclcpp::get_logger("Map"), "Finish updateMap");

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

    void fieldMission(const Path& path, const Drone& drone, Mission& mission) {
        assert(!path.isEmpty());
       
        double timeStart = path.vertexes[0]->getTimeOpen(path.vertexes[1]->getId());

        double length = 0.0;

        for (size_t i = 1; i < path.size() - 1; ++i) {
            length += path.vertexes[i-1]->getLength(path.vertexes[i]->getId());
            double timeFly = timeStart + length / drone.getVoptimal();
            double timeOpen = path.vertexes[i]->getTimeOpen(path.vertexes[i+1]->getId());
            if (timeFly < timeOpen + getSafetyTime()) {
                double dtime = (timeOpen - timeFly) + getSafetyTime();
                timeStart += dtime;
            }
        }

        mission.timeStart = timeStart;
        mission.timeFinish = timeStart + path.getDistance() / drone.getVoptimal();
        mission.vertexes.push_back(path.vertexes[0]->getId());
        for (size_t i = 1; i < path.size(); ++i) {
            mission.velocities.push_back(drone.getVoptimal());
            mission.vertexes.push_back(path.vertexes[i]->getId());
        }
    }

    double getSafetyTime() const {
        return 5.0; // Placeholder for safety time calculation
    }

    Path findOptimalPath(const std::list<Path>& paths, Drone drone, 
                         OptimalPathMode mode) {
        assert(!paths.empty());

        if (mode == OptimalPathMode::MIN_DISTANCE) {
            return findMinimalDistancePath(paths);
        }
        
        return Path();
    }

    const Path& findMinimalDistancePath(const std::list<Path>& paths) const {
        assert(!paths.empty());
        return *std::min_element(paths.begin(), paths.end(),
            [](const Path& a, const Path& b) {
                return a.getDistance() < b.getDistance();
            });
    }

    void generatePaths(const std::string& id_from, const std::string& id_to, 
                       std::list<Path>& paths) {
        assert(vertexes.find(id_from) != vertexes.end());
        assert(vertexes.find(id_to) != vertexes.end());
        
        Path curPath;

        recursiveGeneratePath(id_from, id_to, paths, curPath);
    }

private:
    void recursiveGeneratePath(const std::string& id_from, const std::string& id_to, 
                               std::list<Path>& paths, Path curPath) {
        assert(vertexes.find(id_from) != vertexes.end());
        assert(vertexes.find(id_to) != vertexes.end());

        // std::cout << "Recursive call: " << id_from << " -> " << id_to << std::endl;
        // std::cout << "Current path: ";
        // for (const auto& vertex: curPath.vertexes) {
        //     std::cout << vertex->getId() << " ";
        // }
        // std::cout << std::endl;

        if (id_from == id_to) {
            curPath.addVertex(vertexes[id_from]);
            paths.push_back(curPath);
            return;
        }

        if (vertexes[id_from]->isHasNeighbors() &&
            curPath.isNotInPath(id_from)) 
        {
            // std::cout << "Adding vertex: " << id_from << std::endl;
            curPath.addVertex(vertexes[id_from]);

            for (const auto& neighbor: vertexes[id_from]->getNeighbors()) {
                // std::cout << "Checking neighbor: " << neighbor->getId() << std::endl;
                if (!neighbor->isCanFlyAcross() && 
                    neighbor->getId() != id_to) { continue; }
                Path newPath = curPath;
                recursiveGeneratePath(neighbor->getId(), id_to, paths, newPath);
            }
            return;
        }
    }

private:
    std::unordered_map<std::string, std::shared_ptr<Vertex>> vertexes;    
};