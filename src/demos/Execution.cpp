#ifndef EXECFUNCS
#define EXECFUNCS

#include <geometry_msgs/Twist.h>
#include "../src/discrete/MoveInFreeSpaceOptimizer.cpp"

tuple<ControlSequence, double> moveRobotBackWithTrajectoryOptimization(vector<string> movableObstacleNames,
                                                                       vector<string> staticObstacleNames,
                                                                       const double planeHighX,
                                                                       const double planeLowX,
                                                                       const double planeHighY,
                                                                       const double planeLowY,
                                                                       const double tableZ) {

    printf("Moving robot backwards, optimization starting...\n");

    // Find the opposite direction of the robot's end-effector.
    auto endEffectorPosition = globalMujocoHelper->getSitePosition(
            "ee_point_1"); // This is the position of the end-effector.
    double endEffectorX = endEffectorPosition[0];
    double endEffectorY = endEffectorPosition[1];

    auto site2 = globalMujocoHelper->getSitePosition("ee_point_2");
    double site2_x = site2[0];
    double site2_y = site2[1];

    // Direction vector
    double eeVectorX = site2_x - endEffectorX;
    double eeVectorY = site2_y - endEffectorY;
    vector<double> directionVector = {eeVectorX, eeVectorY};
    vector<double> unitDirectionVector = globalMujocoHelper->unitVectorOf(directionVector);
    vector<double> oppositeUnitDirectionVector = {-unitDirectionVector[0], -unitDirectionVector[1]};

    oppositeUnitDirectionVector[0] *= 0.07;
    oppositeUnitDirectionVector[1] *= 0.07;

    int TRAJECTORY_ROLLOUTS = 5;
    int MAX_ITERATIONS = 5;
    double COST_THRESHOLD = 10.0;
    int TRAJECTORY_DURATION = 1;
    int CONTROL_SEQUENCE_STEPS = 5;
    vector<double> VARIANCE_VECTOR = {0.03, 0.03, 0.02};

    MoveInFreeSpaceOptimizer optimizer(globalMujocoHelper, planeHighX, planeLowX, planeHighY, planeLowY, tableZ);
    optimizer.setNumberOfNoisyTrajectoryRollouts(TRAJECTORY_ROLLOUTS);
    optimizer.setMaxIterations(MAX_ITERATIONS);
    optimizer.setCostThreshold(COST_THRESHOLD);
    optimizer.setTrajectoryDuration(TRAJECTORY_DURATION);
    optimizer.setSamplingVarianceVector(&VARIANCE_VECTOR);
    optimizer.setStaticObstacleNames(&staticObstacleNames);
    optimizer.setMovableObstacleNames(&movableObstacleNames);
    optimizer.setControlSequenceSteps(CONTROL_SEQUENCE_STEPS);

    double initialX = globalMujocoHelper->getRobotXpos();
    double initialY = globalMujocoHelper->getRobotYpos();
    double desiredX = initialX + oppositeUnitDirectionVector[0];
    double desiredY = initialY + oppositeUnitDirectionVector[1];
    double desiredYaw = 0.0;

    optimizer.setPointToMoveTo(desiredX, desiredY, desiredYaw);
    State startState = globalMujocoHelper->getState();
    optimizer.setInitialState(startState);

    Result output = optimizer.optimize();
    bool isSuccessful = output.isSuccessful();

    ControlSequence controlSequence(CONTROL_SEQUENCE_STEPS);
    double duration = optimizer.getActionDuration();

    if (isSuccessful) {
        controlSequence = output.getControlSequence();
    } else {
        printf("Failed to move backwards.\n");
    }

    return make_tuple(controlSequence, duration);
}

void executeRidgebackControlSimulationNoise(Control currentControl, double propagationStepSize,
                                            const vector<string>& movableObjects) {
    vector<double> initialGripperJointValues = globalMujocoHelper->getCurrentJointValuesForGripper();
    vector<double> finalGripperJointValues = globalMujocoHelper->getJointValuesForGripper(
            currentControl.getGripperDOF()); // Fully-closed

    double steps = propagationStepSize / globalMujocoHelper->getTimeStep();

    vector<double> diffGripperJointValues = finalGripperJointValues;
    for (unsigned int i = 0; i < initialGripperJointValues.size(); ++i) {
        diffGripperJointValues[i] -= initialGripperJointValues[i];
    }

    std::random_device rand_dev;
    std::mt19937 linearX(rand_dev());
    std::mt19937 linearY(rand_dev());
    std::normal_distribution<double> d1(0.0, 0.02);
    std::normal_distribution<double> d2(0.0, 0.02);

    for (int step = 0; step < steps; ++step) {
        globalMujocoHelper->setRobotVelocity(currentControl.getLinearX(), currentControl.getLinearY(),
                                             currentControl.getAngularZ());

        // Set gripper DOF
        vector<double> stepGripperJointValues = initialGripperJointValues;
        for (unsigned int i = 0; i < stepGripperJointValues.size(); ++i) {
            stepGripperJointValues[i] += diffGripperJointValues[i] * (step / steps);
        }

        globalMujocoHelper->setGripperJointValues(stepGripperJointValues);

        for (const auto& objectName : movableObjects) {
            double objectLinearX = get<0>(globalMujocoHelper->getBodyVelocity(objectName));
            double objectLinearY = get<1>(globalMujocoHelper->getBodyVelocity(objectName));
            if (objectLinearX > 0.05 || objectLinearY > 0.05) {
                double randX = d1(linearX);
                double randY = d2(linearY);
                globalMujocoHelper->setBodyVelocity(objectName, objectLinearX + randX, objectLinearY + randY, 0.0, 0.0,
                                                    0.0, 0.0);
            }
        }

        globalMujocoHelper->step();
    }

    finishedExecution = true;
}

void executeSolutionFast(const ControlSequence& controlSequence, double propagationStepSize) {
    for (Control currentControl : controlSequence.getControls()) {
        vector<double> initialGripperJointValues = globalMujocoHelper->getCurrentJointValuesForGripper();
        vector<double> finalGripperJointValues = globalMujocoHelper->getJointValuesForGripper(
                currentControl.getGripperDOF()); // Fully-closed

        double steps = propagationStepSize / globalMujocoHelper->getTimeStep();

        vector<double> diffGripperJointValues = finalGripperJointValues;
        for (unsigned int i = 0; i < initialGripperJointValues.size(); ++i) {
            diffGripperJointValues[i] -= initialGripperJointValues[i];
        }

        for (int step = 0; step < steps; ++step) {
            globalMujocoHelper->setRobotVelocity(currentControl.getLinearX(), currentControl.getLinearY(),
                                                 currentControl.getAngularZ());

            // Set gripper DOF
            vector<double> stepGripperJointValues = initialGripperJointValues;
            for (unsigned int i = 0; i < stepGripperJointValues.size(); ++i) {
                stepGripperJointValues[i] += diffGripperJointValues[i] * (step / steps);
            }

            globalMujocoHelper->setGripperJointValues(stepGripperJointValues);

            globalMujocoHelper->step();
            std::this_thread::sleep_for(std::chrono::nanoseconds(500));
        }
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    finishedExecution = true;
}

#endif
