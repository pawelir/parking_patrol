#ifndef PARKING_PATROL_SPOT_FINDER_HPP
#define PARKING_PATROL_SPOT_FINDER_HPP

#include <vector>

namespace parkingpatrol {

class SpotFinder {

 public:
  SpotFinder(double spot_depth, double spot_width);

  bool find_spot(const std::vector<float> &laser_readings);

 private:
  double spot_depth_;
  double spot_width_;

};

}// namespace parkingpatrol

#endif//PARKING_PATROL_SPOT_FINDER_HPP
