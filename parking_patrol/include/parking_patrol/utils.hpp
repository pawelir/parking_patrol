#ifndef PARKING_PATROL_PARKING_UTILS_HPP
#define PARKING_PATROL_PARKING_UTILS_HPP

namespace parkingpatrol
{
struct SpotDetectionPose
{
  SpotDetectionPose(float x, float y, float theta) : x(x), y(y), theta(theta) {}
  float x;
  float y;
  float theta;
};

enum StatusCode { REJECTED, SUCCEDED };

namespace direction
{
static constexpr char UP[] = "UP";
static constexpr char DOWN[] = "DOWN";
static constexpr char TOLERANCE[] = "TOLERANCE";
}  // namespace direction

namespace side
{
static constexpr char LEFT[] = "LEFT";
static constexpr char RIGHT[] = "RIGHT";
}  // namespace side

}  // namespace parkingpatrol

#endif  // PARKING_PATROL_PARKING_UTILS_HPP
