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
            FINDING_MAX_VEL, //Automatically find max vel
            TUNING,
            RECENTER,
            CALCULATING
        };

        MotorAutotuner(std::string name, bool positionUnique, double maxVolts);

        void Start();
        void SetCurrentPose(Poses::Pose1D pose);
        void SetCurrentPose(Poses::MotorPose pose);
        double GetVoltage();
        void Finish();
        void Pause();

        void SetMin(double min);
        void SetMax(double max);
        void AddBounds(double pos);
        void SetDegreeTestingBounds(double degree);

        void SetDensity(int density);

    private:
        Config config_;

        State state_;
        void StateCalculations();
        
        Poses::MotorPose currPose_;
        Poses::MotorPose prevPose_;
        Poses::MotorPose targetPose_;

        bool foundMaxVel_;
        double upVel_, downVel_;
        std::vector<Poses::MotorPose> velFindingPoses_;
        double FindVelRange();

        double Tune();

        double Recenter();

        double degreeTestingBounds_ = 12.0; //Testing will padded 1/12th the way from the absolute bounds
        Bounds bounds_;
        void CalcBounds();

        std::unordered_map<Coordinate, std::vector<Poses::MotorPose>, pair_hash> data_; // Coordinate -> vector of poses
        int density_ = 50; //Number of cells on each axis
        double gridSizePos_; //Size of each coordinate
        double gridSizeVel_;
        Coordinate GetCoordinate(Poses::MotorPose pose);
};