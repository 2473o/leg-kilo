#ifndef LEG_KILO_OPTIONS_H
#define LEG_KILO_OPTIONS_H

#include <atomic>
#include <string>
namespace legkilo {
namespace options {

extern std::string kLidarTopic;
extern std::string kKinematicTopic;
extern std::string kKinematicType;
extern std::string kImuTopic;

extern std::string kOdomTopic;
extern std::string kOdomFrameId;
extern std::string kBaseFrameId;

extern bool kKinAndImuUse;  // use kin. and imu together
extern bool kImuUse;        // only use imu
extern std::atomic_bool FLAG_EXIT;
extern bool kRedundancy;  // for legkilo dataset

}  // namespace options
}  // namespace legkilo

#endif  // LEG_KILO_OPTIONS_H