#include <gtest/gtest.h>
#include "navigator2/map.hpp"

class MapTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Настройка тестовой карты
        map.addVertex("d1", {80,158,0});
        map.addVertex("d2", {251,414,0});
        map.addVertex("d3", {468,40,0});
        map.addVertex("d4", {677,307,0});

        map.addVertex("p1", {508,154,0});
        map.addVertex("p2", {223,110,0});
        map.addVertex("p3", {267,221,0});
        map.addVertex("p4", {80,303,0});
        map.addVertex("p5", {241,317,0});
        map.addVertex("p6", {424,353,0});

        
        map.addEdge("d1", "p3");
        map.addEdge("d1", "p4");
        map.addEdge("p2", "d1");
        map.addEdge("d3", "p2");
        map.addEdge("d3", "p1");
        map.addEdge("p3", "p2");
        map.addEdge("p3", "d3");
        map.addEdge("p3", "p4");
        map.addEdge("p3", "p6");
        map.addEdge("p1", "p3");
        map.addEdge("d4", "p1");
        map.addEdge("d4", "p3");
        map.addEdge("p4", "d2");
        map.addEdge("p5", "p3");
        map.addEdge("d2", "p5");
        map.addEdge("d2", "p6");
        map.addEdge("p6", "d4");
        map.addEdge("p6", "p5");
    }

    Map map;
};

TEST(VertexTest, BasicFunctionality) {
    Vertex::Point position{1.0, 2.0, 3.0};
    Vertex vertex("p1", position);
    
    EXPECT_EQ(vertex.getId(), "p1");
    EXPECT_EQ(vertex.getPosition(), position);
    EXPECT_FALSE(vertex.isHasNeighbors());
    EXPECT_TRUE(vertex.isCanFlyAcross());
}

TEST(VertexTest, NeighborOperations) {
    auto p1 = std::make_shared<Vertex>("p1", Vertex::Point{0,0,0});
    auto p2 = std::make_shared<Vertex>("p2", Vertex::Point{1,0,0});
    
    p1->addNeighbor(p2, 10.0);
    
    EXPECT_TRUE(p1->isHasNeighbors());
    EXPECT_EQ(p1->getNeighbors().size(), 1);
    EXPECT_DOUBLE_EQ(p1->getLength("p2"), 1.0);
    EXPECT_DOUBLE_EQ(p1->getTimeOpen("p2"), 10.0);
}

TEST(PathTest, BasicOperations) {
    Path path;
    EXPECT_TRUE(path.isEmpty());
    EXPECT_TRUE(path.isNotInPath("p1"));
    
    auto p1 = std::make_shared<Vertex>("p1", Vertex::Point{0,0,0});
    path.vertexes.push_back(p1);
    path.visited.insert("p1");
    
    EXPECT_FALSE(path.isEmpty());
    EXPECT_FALSE(path.isNotInPath("p1"));
    EXPECT_EQ(path.size(), 1);
}

TEST(DroneTest, VelocitySettings) {
    Drone drone("p1", 10.0, 1.0, 10.0);
    
    EXPECT_DOUBLE_EQ(drone.getVmax(), 10.0);
    EXPECT_DOUBLE_EQ(drone.getVmin(), 1.0);
    EXPECT_DOUBLE_EQ(drone.getVoptimal(), 5.5);
    
    drone.setVmax(20.0);
    EXPECT_DOUBLE_EQ(drone.getVoptimal(), 10.5);
}

TEST_F(MapTest, VertexAndEdgeOperations) {
    EXPECT_EQ(map.size(), 10);
}

TEST_F(MapTest, PathGeneration) {
    std::list<Path> paths;
    map.generatePaths("d1", "d2", paths);
    
    EXPECT_EQ(paths.size(), 2);

    for (const auto& path : paths) {
        EXPECT_EQ(path.vertexes.front()->getId(), "d1");
        EXPECT_EQ(path.vertexes.back()->getId(), "d2");
    }
}

TEST_F(MapTest, OptimalPathSelection) {
    std::list<Path> paths;
    map.generatePaths("d1", "d2", paths);
    
    Drone drone("d1", 11.0, 1.0, 10.0);
    Path optimalPath = map.findOptimalPath(paths, drone, OptimalPathMode::MIN_DISTANCE);
    
    EXPECT_FALSE(optimalPath.isEmpty());    
    EXPECT_EQ(optimalPath.size(), 3);
    EXPECT_EQ(optimalPath.vertexes[0]->getId(), "d1");
    EXPECT_EQ(optimalPath.vertexes[1]->getId(), "p4");
    EXPECT_EQ(optimalPath.vertexes[2]->getId(), "d2");
}

TEST_F(MapTest, MissionGeneration) {
    Drone drone("p1", 11.0, 1.0, 10.0);
    Mission mission = map.generateMission("d1", "d2", drone, OptimalPathMode::MIN_DISTANCE);
    
    EXPECT_EQ(mission.vertexes.size(), 3);
    EXPECT_EQ(mission.velocities.size(), 2);
    
    // Первая и последняя вершины должны совпадать с началом и концом
    EXPECT_EQ(mission.vertexes.front(), "d1");
    EXPECT_EQ(mission.vertexes.back(), "d2");
    
    // Все скорости должны быть оптимальными
    for (double v : mission.velocities) {
        EXPECT_DOUBLE_EQ(v, drone.getVoptimal());
    }
}

// TEST_F(MapTest, MapUpdateAfterMission) {
//     Drone drone("d1", 11.0, 1.0);
//     Mission mission = map.generateMission("d1", "d2", drone, OptimalPathMode::MIN_DISTANCE);
    
//     // Проверяем что времена открытия обновились
//     for (size_t i = 0; i < mission.vertexes.size() - 1; ++i) {
//         std::string current = mission.vertexes[i];
//         std::string next = mission.vertexes[i+1];
//         double timeOpen = map.vertexes[current]->getTimeOpen(next);
//         EXPECT_GT(timeOpen, 0);
//     }
// }

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}