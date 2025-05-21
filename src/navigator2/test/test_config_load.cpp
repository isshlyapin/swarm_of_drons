#include <string>
#include <filesystem>

#include <gtest/gtest.h>

#include "navigator2/map.hpp"
#include "navigator2/mission_manager.hpp"

namespace fs = std::filesystem;

/// Получить путь к тестовому файлу относительно текущего файла
std::string get_test_file(const std::string &relative_path)
{
  fs::path test_dir = fs::path(__FILE__).parent_path() / "../../../tests/test5";
  return (test_dir / relative_path).lexically_normal().string();
}

TEST(MapConfigLoad, LoadGraphFromCSV) {
    Map map;
    map.loadGraphFromCSV(get_test_file("graph.csv"));

    EXPECT_EQ(map.size(), 4);
    EXPECT_NO_THROW(map.getVertexPosition("d1"));
    EXPECT_NO_THROW(map.getVertexPosition("d2"));
    EXPECT_NO_THROW(map.getVertexPosition("p4"));
    EXPECT_NO_THROW(map.getVertexPosition("p5"));

    EXPECT_TRUE(Map::Point(80, 158,0).isApprox(map.getVertexPosition("d1"), 1e-10));
    EXPECT_TRUE(Map::Point(251,414,0).isApprox(map.getVertexPosition("d2"), 1e-10));
    EXPECT_TRUE(Map::Point(80, 303,0).isApprox(map.getVertexPosition("p4"), 1e-10));
    EXPECT_TRUE(Map::Point(241,317,0).isApprox(map.getVertexPosition("p5"), 1e-10));
}

TEST(MapConfigLoad, LoadEdgesFromCSV) {
    Map map;
    map.loadGraphFromCSV(get_test_file("graph.csv"));
    map.loadEdgesFromCSV(get_test_file("edges.csv"));

    EXPECT_EQ(map.size(), 4);
    EXPECT_TRUE(map.getVertexes().at("d1")->isHasNeighbors());
    EXPECT_TRUE(map.getVertexes().at("d2")->isHasNeighbors());
    EXPECT_TRUE(map.getVertexes().at("p4")->isHasNeighbors());
    EXPECT_FALSE(map.getVertexes().at("p5")->isHasNeighbors());

    EXPECT_EQ(map.getVertexes().at("d1")->getNeighbors().size(), 1);
    EXPECT_EQ(map.getVertexes().at("d2")->getNeighbors().size(), 1);
    EXPECT_EQ(map.getVertexes().at("p4")->getNeighbors().size(), 1);

    EXPECT_NO_THROW(map.getVertexes().at("d1")->getNeighbors().at("p4"));
    EXPECT_NO_THROW(map.getVertexes().at("d2")->getNeighbors().at("p5"));
    EXPECT_NO_THROW(map.getVertexes().at("p4")->getNeighbors().at("d2"));
}

TEST(MissionManagerConfigLoad, LoadMissionsFromCSV) {
    MissionManager mm;
    mm.loadMissionsFromCSV(get_test_file("missions.csv"));

    EXPECT_FALSE(mm.empty());

    EXPECT_EQ(mm.size(), 3);
    EXPECT_EQ(mm.topInputMission().id_from, "d2");
    EXPECT_EQ(mm.topInputMission().id_to, "d1");
    EXPECT_DOUBLE_EQ(mm.topInputMission().time_appearance, 1);
    mm.popInputMission();
    
    EXPECT_EQ(mm.topInputMission().id_from, "d2");
    EXPECT_EQ(mm.topInputMission().id_to, "d1");
    EXPECT_DOUBLE_EQ(mm.topInputMission().time_appearance, 2);
    mm.popInputMission();

    EXPECT_EQ(mm.topInputMission().id_from, "d1");
    EXPECT_EQ(mm.topInputMission().id_to, "d2");
    EXPECT_DOUBLE_EQ(mm.topInputMission().time_appearance, 3);
    mm.popInputMission();

    EXPECT_TRUE(mm.empty());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
