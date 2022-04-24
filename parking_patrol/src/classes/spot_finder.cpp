#include <parking_patrol/classes/spot_finder.hpp>
#include <algorithm>
namespace parkingpatrol {

SpotFinder::SpotFinder(double spot_depth, double spot_width)
  : spot_depth_(spot_depth),
    spot_width_(spot_width) {

}

bool SpotFinder::find_spot(const std::vector<float> &laser_readings) {
  ;
  // return std::min_element(laser_readings.begin(), laser_readings.end()) >= spot_depth_;
}

}// namespace parkingpatrol
