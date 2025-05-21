#pragma once 

#include <string>
#include <memory>
#include <unordered_map>

#include <Eigen/Geometry>

class Vertex {
public:
    using Point = Eigen::Vector3d;

    Vertex(std::string vertexId, Point position)
        : id(std::move(vertexId)), position(std::move(position)) {}

    [[nodiscard]] bool isCanFlyAcross() const { return id[0] == 'p'; }

    [[nodiscard]] bool isHasNeighbors() const { return !neighbors.empty(); }

    [[nodiscard]] const std::string& getId() const { return id; }

    [[nodiscard]] const Point& getPosition() const { return position; }
    
    [[nodiscard]] const std::unordered_map<std::string, std::shared_ptr<Vertex>>& 
        getNeighbors() const { return neighbors; }
    
    [[nodiscard]] double getLength(const std::string& vertexId) const {
        return lengths.at(vertexId);
    }

    [[nodiscard]] double getTimeOpen(const std::string& vertexId) const {
        return timesOpen.at(vertexId);
    }

    void setTimeOpen(const std::string& vertexId, double timeOpen) {
        timesOpen[vertexId] = timeOpen;
    }

    void addNeighbor(const std::shared_ptr<Vertex>& neighbor, double timeOpen = 0.0) {
        lengths[neighbor->getId()]   = (neighbor->getPosition() - position).norm();
        neighbors[neighbor->getId()] = neighbor;
        timesOpen[neighbor->getId()] = timeOpen;
    }

    [[nodiscard]] bool operator==(const Vertex& other) const {
        return id == other.id;
    }

    [[nodiscard]] bool operator!=(const Vertex& other) const {
        return !(*this == other);
    }

private:
    std::string id;
    Point position;
    std::unordered_map<std::string, double> lengths;
    std::unordered_map<std::string, double> timesOpen;
    std::unordered_map<std::string, std::shared_ptr<Vertex>> neighbors;
};
