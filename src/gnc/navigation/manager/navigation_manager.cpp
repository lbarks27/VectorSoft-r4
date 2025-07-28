#include "navigation_manager.h"

namespace GNC {

NavigationManager::NavigationManager()
    : current_source_(NavSource::CUBE)
{}

void NavigationManager::selectSource(NavSource source) {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    current_source_ = source;
}

GNC::NavSource NavigationManager::getCurrentSource() {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return current_source_;
}

void NavigationManager::update() {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    switch(current_source_) {
        case NavSource::CUBE:
            nav_state_ = nav_cube_.updateAndGet();
            break;
        case NavSource::DEADRECKON:
            nav_state_ = nav_deadreckon_.updateAndGet();
            break;
        default:
            nav_state_.valid = false;
            break;
    }
}

GNC::NavState NavigationManager::getNavState() {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return nav_state_;
}

}