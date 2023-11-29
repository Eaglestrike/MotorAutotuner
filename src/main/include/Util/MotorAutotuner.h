#pragma once 

#include <string>
#include <unordered_map>
#include <vector>

#include <Util/HashFunction.hpp>
#include <Util/Poses.h>

class MotorAutotuner{
    using Coordinate = std::pair<int, int>; // position, velocity
    public:
        struct Config{
            std::string name;
            bool positionUnique;
            double maxVolts;
        };

        struct Bounds{
            double min; //Absolute min of the mechanism
            double max; //Absolute max of the mechanism
            double center = 0;
            double testMin = 0; //Testing min
            double testMax = 0; //Testing max
        };

        enum State{
            IDLE,
            TUNING,
            RECENTER_FROM_MAX,
            RECENTER_FROM_MIN
        };

        MotorAutotuner(std::string name, bool positionUnique, double maxVolts);

        void SetCurrentPose(Poses::Pose1D pose);
        void SetCurrentPose(Poses::MotorPose pose);
        double GetVoltage();

        void SetMin(double min);
        void SetMax(double max);
        void AddBounds(double pos);

    private:
        Config config_;

        State state_;

        Poses::MotorPose currPose_;

        double degreeTestingBounds_ = 10.0; //Testing will be 1/10th the way from the absolute bounds
        Bounds bounds_;
        void CalcBounds();

        std::unordered_map<Coordinate, std::vector<Poses::MotorPose>, pair_hash> data_; // Coordinate -> vector of poses
        double gridSizePos; //Size of each coordinate
        double gridSizeVel;
        Coordinate GetCoordinate(Poses::MotorPose pose);
};