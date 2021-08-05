#ifndef PUSHTRAJOPT
#define PUSHTRAJOPT

#include "../TrajectoryOptimiserBase.cpp"

class PushOptimizer : public OptimizerBase {
public:
    PushOptimizer(shared_ptr<MujocoHelper> mujocoHelper,
                  double plane_high_x,
                  double plane_low_x,
                  double plane_high_y,
                  double plane_low_y,
                  double table_z) :
            OptimizerBase(mujocoHelper),
            _PLANE_HIGH_X(plane_high_x),
            _PLANE_LOW_X(plane_low_x),
            _PLANE_HIGH_Y(plane_high_y),
            _PLANE_LOW_Y(plane_low_y),
            _TABLE_Z(table_z) {
        _USE_PROBABILISTIC_UPDATE = false;
    }

    void setGoalObjectName(string name) {
        _goalObjectName = name;
    }

    void setGoalRegion(double x, double y, double radius) {
        _goalRegionCentroidX = x;
        _goalRegionCentroidY = y;
        _goalRegionRadius = radius;
    }

private:
    bool isGoalAchieved(State state) {
        return checkIfObjectIsNearTheGoalRegion(state);
    }

    double getDistanceOfObjectToRegion(shared_ptr<MujocoHelper> mujocoHelper) {
        double goalObjectX = mujocoHelper->getBodyXpos(_goalObjectName);
        double goalObjectY = mujocoHelper->getBodyYpos(_goalObjectName);

        double deltaX = goalObjectX - _goalRegionCentroidX;
        double deltaY = goalObjectY - _goalRegionCentroidY;

        return sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    double getDistanceOfObjectToRegion() {
        return getDistanceOfObjectToRegion(_mujocoHelper);
    }

    bool checkIfObjectIsNearTheGoalRegion(State state) {
        setMuJoCoTo(state);
        double distance = getDistanceOfObjectToRegion();
        resetToInitialState();
        return distance < _goalRegionRadius;
    }

    CostSequence cost(StateSequence stateSequence, ControlSequence controlSequence) {
        return cost(stateSequence, controlSequence, _mujocoHelper);
    }

    CostSequence cost(StateSequence stateSequence, ControlSequence /* controlSequence */, shared_ptr<MujocoHelper> mujocoHelper) {
        if (!_movableObstacleNames)
            throw std::invalid_argument("You should set a list of movable obstacle names!");

        CostSequence costSequence(_n); // We don't have a cost for the first step.
        for (unsigned int i = 0; i < stateSequence.size(); ++i) {
            if (i == stateSequence.size() - 1) { // Final state
                auto currentState = stateSequence.getState(i);
                double cost1 = distanceToGoalCost(currentState, mujocoHelper);
                double cost2 = intermediateCosts(currentState, mujocoHelper);
                costSequence.addCost(cost1 + cost2);
            } else {
                auto currentState = stateSequence.getState(i);
                double cost1 = intermediateCosts(currentState, mujocoHelper);
                costSequence.addCost(cost1);
            }
        }

        return costSequence;
    }

    double intermediateCosts(State currentState, shared_ptr<MujocoHelper> mujocoHelper) {
        double edge = edgeCost(currentState, mujocoHelper);
        double highForce = highForceToStaticObstacleCost(currentState, mujocoHelper);

        double collision = 0.0;
        if (_staticObstacleNames)
            collision = collisionCost(currentState, mujocoHelper);

        return edge + collision + highForce;
    }

    double distanceToGoalCost(State currentState, shared_ptr<MujocoHelper> mujocoHelper) {
        setMuJoCoTo(currentState, mujocoHelper);
        double distanceOfGoalObjectToGoalRegion = getDistanceOfObjectToRegion(mujocoHelper);

        if (distanceOfGoalObjectToGoalRegion < _goalRegionRadius) {
            return 0.0;
        }

        double costValue = _REACHED_GOAL_WEIGHT * distanceOfGoalObjectToGoalRegion;

        return costValue;
    }

    float edgeCost(State state, shared_ptr<MujocoHelper> mujocoHelper) {
        setMuJoCoTo(state, mujocoHelper);

        for (unsigned int i = 0; i < _movableObstacleNames->size(); ++i) {
            double objectXpos = mujocoHelper->getBodyXpos(_movableObstacleNames->at(i));
            double objectYpos = mujocoHelper->getBodyYpos(_movableObstacleNames->at(i));

            if (objectXpos > _PLANE_HIGH_X || objectXpos < _PLANE_LOW_X)
                return _EDGE_COST;

            if (objectYpos > _PLANE_HIGH_Y || objectYpos < _PLANE_LOW_Y)
                return _EDGE_COST;
        }

        return 0.0;
    }

    float highForceToStaticObstacleCost(State currentState, shared_ptr<MujocoHelper> mujocoHelper) {
        setMuJoCoTo(currentState, mujocoHelper);

        if (mujocoHelper->isHighForceAppliedToShelf()) {
            return _HIGH_FORCE_COST;
        }

        return 0.0;
    }

    float disturbanceCost(State currentState, State nextState, shared_ptr<MujocoHelper> mujocoHelper) {
        double costValue = 0.0;

        setMuJoCoTo(nextState, mujocoHelper);
        vector<tuple<double, double>> postPositions;
        for (unsigned int i = 0; i < _movableObstacleNames->size(); ++i) {
            double objectXpos = mujocoHelper->getBodyXpos(_movableObstacleNames->at(i));
            double objectYpos = mujocoHelper->getBodyYpos(_movableObstacleNames->at(i));
            postPositions.push_back(make_tuple(objectXpos, objectYpos));
        }

        setMuJoCoTo(currentState, mujocoHelper);
        int index = 0;

        for (unsigned int i = 0; i < _movableObstacleNames->size(); ++i) {
            double preObjectXpos = mujocoHelper->getBodyXpos(_movableObstacleNames->at(i));
            double preObjectYpos = mujocoHelper->getBodyYpos(_movableObstacleNames->at(i));

            double postObjectXpos = get<0>(postPositions[index]);
            double postObjectYpos = get<1>(postPositions[index]);

            double deltaX = postObjectXpos - preObjectXpos;
            double deltaY = postObjectYpos - preObjectYpos;
            costValue += sqrt(deltaX * deltaX + deltaY * deltaY);
            index++;
        }

        return _DISTURBANCE_WEIGHT * (costValue * costValue);
    }

    float collisionCost(State currentState, shared_ptr<MujocoHelper> mujocoHelper) {
        setMuJoCoTo(currentState, mujocoHelper);
        for (unsigned int i = 0; i < _staticObstacleNames->size(); ++i) {
            if (mujocoHelper->isRobotInContact(_staticObstacleNames->at(i))) {
                return _COLLISION_COST;
            }
        }

        return 0.0;
    }

    ControlSequence createCandidateControlSequence() {
        double goalObjectX = _goalRegionCentroidX;
        double goalObjectY = _goalRegionCentroidY;

        auto endEffectorPosition = _mujocoHelper->getSitePosition(
                "ee_point_1"); // This is the position of the end-effector.
        double endEffectorX = endEffectorPosition[0];
        double endEffectorY = endEffectorPosition[1];

        auto site2 = _mujocoHelper->getSitePosition("ee_point_2");
        double site2_x = site2[0];
        double site2_y = site2[1];

        // Direction vector
        double eeVectorX = site2_x - endEffectorX;
        double eeVectorY = site2_y - endEffectorY;
        vector<double> directionVector = {eeVectorX, eeVectorY};
        vector<double> unitDirectionVector = _mujocoHelper->unitVectorOf(directionVector);

        // Find the vector of the end-effector to the goal object.
        double eeToGoalX = goalObjectX - endEffectorX;
        double eeToGoalY = goalObjectY - endEffectorY;
        vector<double> eeToGoalVector = {eeToGoalX, eeToGoalY};
        vector<double> unitEeToGoalVector = _mujocoHelper->unitVectorOf(eeToGoalVector);

        // Calculate the angle between them.
        double angle = acos(_mujocoHelper->dotProduct(unitDirectionVector, unitEeToGoalVector)) * 0.08;

        unitEeToGoalVector[0] *= 0.03;
        unitEeToGoalVector[1] *= 0.03;

        double x = eeToGoalX - unitEeToGoalVector[0];
        double y = eeToGoalY - unitEeToGoalVector[1];

        double linearX = x / _trajectoryDuration;
        double linearY = y / _trajectoryDuration;
        double angularZ = angle / _trajectoryDuration;

        // At this stage where we push the object, we could initially
        // approach the object using open-finger such that the object falls
        // inside the fingers and therefore during this pushing we should
        // keep the hand open, if however we approached the object using
        // closed-fingers (pushing sideways) then we should keep the
        // fingers closed during this push. Either way, the current gripper
        // DOF Value (getGripperDOFValue) will give us the appropriate result.
        double gripperDOFValue = _initialState.getGripperDOFValue();

        Control velocities(linearX, linearY, angularZ, gripperDOFValue);
        ControlSequence controlSequence = ControlSequence(_n);
        for (int i = 0; i < _n; ++i) {
            controlSequence.addControl(velocities);
        }

        return controlSequence;
    }

    const double _EDGE_COST = 500;
    const double _ACCELERATION_WEIGHT = 0.000001;
    const double _DISTURBANCE_WEIGHT = 0.000001;
    const double _COLLISION_COST = 500; // Any static obstacle collision.
    const double _REACHED_GOAL_WEIGHT = 5000;
    const double _HIGH_FORCE_COST = 500;

    // Plane limits
    const double _PLANE_HIGH_X;
    const double _PLANE_LOW_X;
    const double _PLANE_HIGH_Y;
    const double _PLANE_LOW_Y;
    const double _TABLE_Z;

    string _goalObjectName = "object_3";
    double _goalRegionCentroidX = 0.0;
    double _goalRegionCentroidY = 0.0;
    double _goalRegionRadius = 0.0;
};

#endif

