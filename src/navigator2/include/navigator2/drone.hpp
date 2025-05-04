#pragma once 

#include <string>

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
    Drone(std::string vertex_id, DroneVMax vmax, 
        DroneVMin vmin, DroneFreeTime free_time)
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
        vmax.value = new_vmax;
        initVoptimal();
    }

    void setVmin(double new_vmin) {
        vmin.value = new_vmin;
        initVoptimal();
    }

    void setFreeTime(double new_free_time) {
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