#ifndef UR_LAUNCH_HARDWARE_MATCH_UR_CALIBRATION_CONSUMER_H_INCLUDED
#define UR_LAUNCH_HARDWARE_MATCH_UR_CALIBRATION_CONSUMER_H_INCLUDED
#include "ros/ros.h"

#include <vector>

#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/primary/robot_state/kinematics_info.h>

#include <ur_calibrated_pose_pub/utils/dh_transformation.h>

namespace ur_launch_hardware
{
    class URCalibrationConsumer : public urcl::comm::IConsumer<urcl::primary_interface::PrimaryPackage>
    {
    public:
        URCalibrationConsumer();
        virtual ~URCalibrationConsumer() = default;

        virtual bool consume(std::shared_ptr<urcl::primary_interface::PrimaryPackage> product);

        bool isCalibrated() const
        {
            return calibrated_;
        }

        std::vector<dh_utils::DHTransformation> getDHTransformationsList();

    private:
        bool calibrated_;

        std::vector<dh_utils::DHTransformation> dh_transformations_list_;
    };
}  // namespace ur_launch_hardware
#endif  // ifndef UR_LAUNCH_HARDWARE_MATCH_UR_CALIBRATION_CONSUMER_H_INCLUDED
