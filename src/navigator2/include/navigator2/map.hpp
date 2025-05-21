#pragma once

#include <set>
#include <list>
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>

#include <Eigen/Geometry>

#include "navigator2/drone.hpp"
#include "navigator2/config.hpp"
#include "navigator2/vertex.hpp"

struct Path {
    [[nodiscard]] bool isNotInPath(const std::string& vertexId) const {
        return visited.find(vertexId) == visited.end();
    }

    [[nodiscard]] bool isEmpty() const { return vertexes.empty(); }

    [[nodiscard]] size_t size() const { return vertexes.size(); }

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

    void addVertex(const std::shared_ptr<Vertex>& vertex) {
        visited.insert(vertex->getId());
        vertexes.push_back(vertex);
    }

    [[nodiscard]] const std::shared_ptr<Vertex>& back() const {
        return vertexes.back();
    }

    [[nodiscard]] const std::shared_ptr<Vertex>& front() const {
        return vertexes.front();
    }

    [[nodiscard]] const std::shared_ptr<Vertex>& operator[](size_t index) const {
        return vertexes[index];
    }

    [[nodiscard]] bool operator==(const Path& other) const {
        if (size() != other.size()) {
            return false;
        }
        for (size_t i = 0; i < size(); ++i) {
            if (vertexes[i]->getId() != other.vertexes[i]->getId()) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool operator!=(const Path& other) const {
        return !(*this == other);
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

class Map {
public:
    using Point = Eigen::Vector3d;

    explicit Map(double safetyTime = 0.0)
        : safetyTime(safetyTime) {};

    void loadGraphFromCSV(const std::string& filePath);
    void loadEdgesFromCSV(const std::string& filePath);

    [[nodiscard]] size_t size() const { return vertexes.size(); }

    static double getEpsilon() { return map_cfg::COORDINATE_TOLERANCE; }

    double getSafetyTime() const { return safetyTime; }

    const Point& getVertexPosition(const std::string& vertexId) const {
        return vertexes.at(vertexId)->getPosition();
    } 

    [[nodiscard]] std::string getVertexId(const Point& position) const;

    void addVertex(const std::string& vertexId, const Point& position);

    void addEdge(const std::string& id_from, const std::string& id_to, 
                 double timeOpen = 0.0);

    Mission generateMission(const std::string& id_from, const std::string& id_to, 
                            const Drone& drone, map_cfg::OptimalPathMode mode);

    void updateMap(const Mission& mission);

    void fieldMission(const Path& path, const Drone& drone, Mission& mission) const;

    static Path findOptimalPath(const std::list<Path>& paths, const Drone& drone, 
                         map_cfg::OptimalPathMode mode);

    static const Path& findMinimalTimePath(const std::list<Path>& paths, 
                                           const Drone& drone);

    static const Path& findMinimalDistancePath(const std::list<Path>& paths);

    void generatePaths(const std::string& id_from, const std::string& id_to, 
                       std::list<Path>& paths);

    [[nodiscard]] const std::unordered_map<std::string, std::shared_ptr<Vertex>>& 
        getVertexes() const { return vertexes; }

private:
    void recursiveGeneratePath(const std::string& id_from, const std::string& id_to, 
                               std::list<Path>& paths, std::list<Path>::iterator& curPathIt);

    double safetyTime;
    std::unordered_map<std::string, std::shared_ptr<Vertex>> vertexes;    
};