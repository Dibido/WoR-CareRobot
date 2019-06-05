#ifndef COLLISION_BEHAVIOR_DATA_HPP
#define COLLISION_BEHAVIOR_DATA_HPP

#include <vector>

namespace sim_torque
{
    struct CollisionBehaviorData
    {
        std::vector<double> mLowerTorqueThresholdsAcceleration_Nm;
        std::vector<double> mUpperTorqueThresholdsAcceleration_Nm;
        std::vector<double> mLowerTorqueThresholdsNominal_Nm;
        std::vector<double> mUpperTorqueThresholdsNominal_Nm;
        std::vector<double> mLowerForceThresholdsAcceleration_N;
        std::vector<double> mUpperForceThresholdsAcceleration_N;
        std::vector<double> mLowerForceThresholdsNominal_N;
        std::vector<double> mUpperForceThresholdsNominal_N;

        CollisionBehaviorData() = default;
        CollisionBehaviorData(
            const std::vector<double>& aLowerTorqueThresholdsAcceleration_Nm,
            const std::vector<double>& aUpperTorqueThresholdsAcceleration_Nm,
            const std::vector<double>& aLowerTorqueThresholdsNominal_Nm,
            const std::vector<double>& aUpperTorqueThresholdsNominal_Nm,
            const std::vector<double>& aLowerForceThresholdsAcceleration_N,
            const std::vector<double>& aUpperForceThresholdsAcceleration_N,
            const std::vector<double>& aLowerForceThresholdsNominal_N,
            const std::vector<double>& aUpperForceThresholdsNominal_N
        );

        ~CollisionBehaviorData() = default;
        CollisionBehaviorData& operator=(const CollisionBehaviorData&) = default;
    };

}

#endif // COLLISION_BEHAVIOR_DATA_HPP