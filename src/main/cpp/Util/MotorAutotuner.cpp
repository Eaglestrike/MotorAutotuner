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
    state_{IDLE},
    currPose_{.pos = 0.0, .vel = 0.0, .acc = 0.0, .volts = 0.0},
    prevPose_{.pos = 0.0, .vel = 0.0, .acc = 0.0, .volts = 0.0},
    targetPose_{.pos = 0.0, .vel = 0.0, .acc = 0.0, .volts = 0.0},
    foundMaxVel_{false},
    upVel_{0.0}, downVel_{0.0},
    bounds_{.min = 0.0, .max = 0.0, .center = 0.0, .testMin = 0.0, .testMax = 0.0},
    gridSizePos_{0.0}, gridSizeVel_{0.0}
{
}

void MotorAutotuner::Start(){
    state_ = FINDING_MAX_VEL;
}

void MotorAutotuner::Pause(){
    state_ = IDLE;
}

void MotorAutotuner::Finish(){
    state_ = CALCULATING;
}

/**
 * Sets the information on the mechanism
 * 
 * Stores the data in the map
*/
void MotorAutotuner::SetCurrentPose(Poses::Pose1D pose){
    SetCurrentPose({
        .pos = pose.pos,
        .vel = pose.vel,
        .acc = pose.acc,
        .volts = currPose_.volts
    }); //Converts to MotorPose
}

/**
 * Sets the information on the mechanism
 * 
 * Stores the data in the map
 * 
 * @param pose pose of the motor or mechanism
*/
void MotorAutotuner::SetCurrentPose(Poses::MotorPose pose){
    prevPose_ = currPose_;
    currPose_ = pose;
    if(state_ == IDLE){ // Don't store data or execute when Idle
        return;
    }
    if(!foundMaxVel_){
        velFindingPoses_.push_back(currPose_);
    }
    else{
        Coordinate coordinate = GetCoordinate(currPose_);
        data_[coordinate].push_back(currPose_); //Store data in map
    }
}

/**
 * Gets the coordinate of the grid that the pose is in
 * x(first) = pos, y(second) = vel
 * 
 * @param pose pose containing position and velocity
*/
MotorAutotuner::Coordinate MotorAutotuner::GetCoordinate(Poses::MotorPose pose){
    int posCoord;
    if(config_.positionUnique){
        posCoord = (int)(pose.pos/gridSizePos_);
    }
    else{
        posCoord = 0;
    }
    int velCoord = (int)(pose.vel/gridSizeVel_);
    return Coordinate(posCoord, velCoord);
}

/**
 * Main function to actually move things
 * Should assign this voltage to the motor/mechanism
 * 
 * @returns the testing voltage
*/
double MotorAutotuner::GetVoltage(){
    StateCalculations();
    return currPose_.volts;
}

/**
 * Does the calculations based on the state
*/
void MotorAutotuner::StateCalculations(){
    switch(state_){
        case IDLE:
            currPose_.volts = 0.0;
            break;
        case FINDING_MAX_VEL:
            currPose_.volts = FindVelRange();
            break;
        case TUNING:
            currPose_.volts = Tune();
            break;
        case RECENTER:
            currPose_.volts = Recenter();
            break;
        case CALCULATING:
            currPose_.volts = 0.0;
            break;
    }
}

/**
 * Finds the maximum velocity of the mechanism
 * 
 * @return target voltage
*/
double MotorAutotuner::FindVelRange(){
    if(currPose_.vel > upVel_){
        upVel_ = currPose_.vel;
    }
    if(currPose_.vel < downVel_){
        downVel_ = currPose_.vel;
    }
    if(true /*is done*/){
        state_ = TUNING;
        foundMaxVel_ = true;
        gridSizeVel_ = (upVel_ - downVel_)/density_; //Find vel grid size
        for(Poses::MotorPose pose : velFindingPoses_){ //Load all the poses into the grid
            Coordinate coordinate = GetCoordinate(pose);
            data_[coordinate].push_back(pose); //Store data in map
        }
        velFindingPoses_.clear();
        return Tune();
    }
}

/**
 * Main tuning function
 * 
 * @return target voltage
*/
double MotorAutotuner::Tune(){
    if(currPose_.pos < bounds_.testMin){
        state_ = RECENTER;
        return Recenter();
    }
    if(currPose_.pos > bounds_.testMax){
        state_ = RECENTER;
        return Recenter();
    }
    return 0.0;
}

/**
 * Recenters the mechanism if it's gone somewhere else
 * 
 * @return target voltage
*/
double MotorAutotuner::Recenter(){
    bool aboveCenter = currPose_.pos > bounds_.center;
    bool pastAboveCenter = prevPose_.pos > bounds_.center;
    if(aboveCenter ^ pastAboveCenter){ //If are opposite, XOR operator
        state_ = TUNING;
        return Tune();
    }
    if(aboveCenter){
        return -config_.maxVolts;
    }
    else{
        return config_.maxVolts;
    }
}

/**
 * Sets the min position of the motor/mechanism
*/
void MotorAutotuner::SetMin(double min){
    if(state_ == IDLE){
        bounds_.min = min;
        CalcBounds();
    }
}

/**
 * Sets the max position of the motor/mechanism
*/
void MotorAutotuner::SetMax(double max){
    if(state_ == IDLE){
        bounds_.max = max;
        CalcBounds();
    }
}

/**
 * Adds the position to the given absolute bounds
 * Can be called periodically to manually set a range w/o inputting values
 * 
 * @param pos position to add
*/
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


void MotorAutotuner::SetDegreeTestingBounds(double degree){
    if(state_ != IDLE){
        degreeTestingBounds_ = degree;
        CalcBounds();
    }
}

/**
 * Internal Function, calculates variables using the current bounds
 * Sets bounds_(center, testMin, testMax) and gridSizepos_
 * 
 * SHOULD NOT BE CALLED IF THE AUTOTUNER IS RUNNING
*/
void MotorAutotuner::CalcBounds(){
    bounds_.center = (bounds_.min + bounds_.max)/2.0;
    double range = bounds_.max - bounds_.min;
    double testPadding;
    if(degreeTestingBounds_ == 0.0){
        testPadding = 0.0;
    }
    else{
        testPadding = range/degreeTestingBounds_;
    }
    bounds_.testMin = bounds_.min + testPadding;
    bounds_.testMax = bounds_.max - testPadding;

    gridSizePos_ = range / density_;
}

void MotorAutotuner::SetDensity(int density){
    density_ = density;
    CalcBounds();
}