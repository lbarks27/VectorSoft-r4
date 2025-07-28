#pragma once

#include "nav_cube.h"
#include "nav_deadreckon.h"

namespace GNC {

// High-level navigation state struct
struct NavState {
    double lat, lon, alt;
    double velN, velE, velD;
    double roll, pitch, yaw;
    double timestamp;
    bool valid;
    // Add more fields as needed
};

enum class NavSource {
    CUBE,
    DEADRECKON
    // Expand as needed
};

class NavigationManager {
public:
    NavigationManager();

    void selectSource(NavSource source);
    NavSource getCurrentSource();

    // Called each cycle to update nav data from selected source
    void update();

    // Thread-safe getter for current nav state
    NavState getNavState();

private:
    NavSource current_source_;
    NavCube nav_cube_;
    NavDeadReckon nav_deadreckon_;
    NavState nav_state_;
    std::mutex nav_mutex_;
};

}