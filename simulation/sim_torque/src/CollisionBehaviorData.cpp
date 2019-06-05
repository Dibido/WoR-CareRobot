#include "sim_torque/CollisionBehaviorData.hpp"


sim_torque::CollisionBehaviorData::CollisionBehaviorData(
            const std::vector<double>& aLowerTorqueThresholdsAcceleration_Nm,
            const std::vector<double>& aUpperTorqueThresholdsAcceleration_Nm,
            const std::vector<double>& aLowerTorqueThresholdsNominal_Nm,
            const std::vector<double>& aUpperTorqueThresholdsNominal_Nm,
            const std::vector<double>& aLowerForceThresholdsAcceleration_N,
            const std::vector<double>& aUpperForceThresholdsAcceleration_N,
            const std::vector<double>& aLowerForceThresholdsNominal_N,
            const std::vector<double>& aUpperForceThresholdsNominal_N
        )
        :   mLowerTorqueThresholdsAcceleration_Nm(aLowerTorqueThresholdsAcceleration_Nm),
            mUpperTorqueThresholdsAcceleration_Nm(aUpperTorqueThresholdsAcceleration_Nm),
            mLowerTorqueThresholdsNominal_Nm(aLowerTorqueThresholdsNominal_Nm),
            mUpperTorqueThresholdsNominal_Nm(aUpperTorqueThresholdsNominal_Nm),
            mLowerForceThresholdsAcceleration_N(aLowerForceThresholdsAcceleration_N),
            mUpperForceThresholdsAcceleration_N(aUpperForceThresholdsAcceleration_N),
            mLowerForceThresholdsNominal_N(aLowerForceThresholdsNominal_N),
            mUpperForceThresholdsNominal_N(aUpperForceThresholdsNominal_N)
        {}