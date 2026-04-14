#include "interface/ros2/options.h"

namespace legkilo {
namespace options {

bool kKinAndImuUse = false;
bool kImuUse = false;
std::atomic_bool FLAG_EXIT{false};
bool kRedundancy = false;

std::string kLidarTopic;
std::string kKinematicTopic;
std::string kKinematicType;
std::string kImuTopic;

std::string kOdomTopic;
std::string kOdomFrameId;
std::string kBaseFrameId;

}  // namespace options
}  // namespace legkilo