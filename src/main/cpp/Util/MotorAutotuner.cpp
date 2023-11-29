#include <Util/MotorAutotuner.h>


/**
 * Constructor
 * 
 * @param name name
 * @param positionUnique if behavior is not dependent of position (gravity usually) or if mechanism is unbounded in game
 * @param maxVolts maximum volts
*/
MotorAutotuner::MotorAutotuner(std::string name, bool positionUnique, double maxVolts):
    config_{
        name = name,
        positionUnique = positionUnique,
        maxVolts = maxVolts
    },
    state_{TUNING},
    data_{}
{

}

void MotorAutotuner::SetCurrentPose(Poses::Pose1D pose){
    SetCurrentPose({
        .pos = pose.pos,
        .vel = pose.vel,
        .acc = pose.acc,
        .volts = currPose_.volts
    });

}

void MotorAutotuner::SetCurrentPose(Poses::MotorPose pose){
    currPose_ = pose;
    if(currPose_.pos > bounds_.testMax){
        state_ = RECENTER_FROM_MAX;
    }
    if(currPose_.pos < bounds_.testMin){
        state_ = RECENTER_FROM_MIN;
    }
    Coordinate coordinate = GetCoordinate(currPose_);
    data_[coordinate].push_back(currPose_);
}

double MotorAutotuner::GetVoltage(){
    switch(state_){
        case IDLE:
            return 0.0;
        case TUNING:
            return currPose_.volts;
        case RECENTER_FROM_MAX:
            return currPose_.volts;
        case RECENTER_FROM_MIN:
            return currPose_.volts;
    }
}

void MotorAutotuner::SetMin(double min){
    if(state_ == IDLE){
        bounds_.min = min;
        CalcBounds();
    }
}

void MotorAutotuner::SetMax(double max){
    if(state_ == IDLE){
        bounds_.max = max;
        CalcBounds();
    }
}

void MotorAutotuner::AddBounds(double pos){
    if(state_ == IDLE){
        if(pos > bounds_.max){
            SetMax(pos);
        }
        if(pos < bounds_.min){
            SetMin(pos);
        }
    }
}

void MotorAutotuner::CalcBounds(){
    bounds_.center = (bounds_.min + bounds_.max)/2.0;
    double testPadding;
    if(degreeTestingBounds_ == 0.0){
        testPadding = 0.0;
    }
    else{
        testPadding = (bounds_.max - bounds_.min)/degreeTestingBounds_;
    }
    bounds_.testMin = bounds_.min + testPadding;
    bounds_.testMax = bounds_.max - testPadding;
}