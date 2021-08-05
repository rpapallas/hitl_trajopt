#ifndef TRAJOPT
#define TRAJOPT

#include <future>
#include <mutex>
#include <utility>
#include "State.cpp"
#include "StateSequence.cpp"
#include "Control.cpp"
#include "ControlSequence.cpp"
#include "CostSequence.cpp"
#include "Result.cpp"
#include "../utils/MujocoHelper.cpp"

class OptimizerBase {
public:
    explicit OptimizerBase(const shared_ptr<MujocoHelper> &mujocoHelper) {
        _mujocoHelper = make_shared<MujocoHelper>(mujocoHelper.get());
        updateActionDuration();
    }

    virtual Result optimize() {
        _result = NullResult(0.0);
        _isOptimizing = true;

        if (!_samplingVarianceVector) {
            throw std::runtime_error("The variance vector parameter was not set. Quiting");
        }

        auto optimizationStart = std::chrono::high_resolution_clock::now();

        // This is an initial candidate assuming to be the best so far.
        auto initialControlSequenceAndCost = getInitialControlSequenceAndItsCost();
        ControlSequence bestControlSequence = get<0>(initialControlSequenceAndCost);
        double bestCost = get<2>(initialControlSequenceAndCost);

        if (bestControlSequence.size() == 0) {
            printf("\033[0;31mNot able to generate a valid, initial control sequence. Optimisation aborted.\033[0m\n");
            return NullResult(0);
        }

        // We set _result from now so we can allow someone externally to visualize the current best trajectory.
        Result initialResult(false, 0.0, bestControlSequence);
        _result = initialResult;

        int iterations = 0;
        bool maxIterationNotReached = iterations < _maxIterations;
        bool costThresholdNotReached = bestCost > _costThreshold;

        while (maxIterationNotReached && costThresholdNotReached) {
            iterations++;

            resetToInitialState();
            auto rollouts = createNoisyTrajectories(bestControlSequence);
            resetToInitialState();

            ControlSequence currentControlSequence = updateTrajectory(bestControlSequence, rollouts);

            auto currentRollout = trajectoryRollout(currentControlSequence);
            auto currentCost = get<1>(currentRollout).sum();

            printf("%d. Prior Cost: %f | Updated Cost: %f\n", iterations, bestCost, currentCost);

            if (currentCost < bestCost) {
                bestControlSequence = currentControlSequence;
                bestCost = currentCost;

                // We set _result from now so we can allow someone externally to visualize the current best trajectory.
                Result currentResult(false, 0.0, bestControlSequence);
                _result = currentResult;
            }

            costThresholdNotReached = bestCost > _costThreshold;
            maxIterationNotReached = iterations < _maxIterations;
        }

        resetToInitialState();

        auto optimizationFinish = std::chrono::high_resolution_clock::now();
        double optimizationElapsedTime = formatDurationToSeconds(optimizationStart, optimizationFinish);
        printf("\033[0;33mOptimization took:\033[0m %f seconds after %d iterations.\n", optimizationElapsedTime,
               iterations);

        auto bestStateSequence = get<0>(trajectoryRollout(bestControlSequence));
        State finalState = bestStateSequence.getState(bestStateSequence.size() - 1);
        bool didReachTheGoal = isGoalAchieved(finalState);

        Result finalResult(didReachTheGoal, optimizationElapsedTime, bestControlSequence);
        _result = finalResult;
        _isOptimizing = false;
        return finalResult;
    }

    tuple<ControlSequence, StateSequence, double> getInitialControlSequenceAndItsCost() {
        ControlSequence initialControlSequence = createCandidateControlSequence();

        auto rollout = trajectoryRollout(initialControlSequence);
        auto stateSequence = get<0>(rollout);
        double costValue = get<1>(rollout).sum();

        return make_tuple(initialControlSequence, stateSequence, costValue);
    }

    // Getters

    double getActionDuration() {
        updateActionDuration();
        return _actionDuration;
    }

    Result getResult() {
        return _result;
    }

    bool isOptimizing() {
        return _isOptimizing;
    }

    bool isProbabilisticUpdate() {
        return _USE_PROBABILISTIC_UPDATE;
    }

    // Setters

    void setInitialState(State initialState) {
        _initialState = initialState;
    }

    void setMovableObstacleNames(vector<string> *obstacleNames) {
        _movableObstacleNames = obstacleNames;
    }

    void setStaticObstacleNames(vector<string> *obstacleNames) {
        _staticObstacleNames = obstacleNames;
    }

    void setMaxIterations(int maxIterations) {
        _maxIterations = maxIterations;
    }

    void setNumberOfNoisyTrajectoryRollouts(int numberOfNoisyTrajectoryRollouts) {
        _numberOfNoisyTrajectoryRollouts = numberOfNoisyTrajectoryRollouts;

        // Create a MuJoCo Helper for each of the K noisy trajectories (for palatalization).
        createLocalMujocoHelpers();
    }

    void setCostThreshold(double costThreshold) {
        _costThreshold = costThreshold;
    }

    void setSamplingVarianceVector(vector<double> *samplingVarianceVector) {
        _samplingVarianceVector = samplingVarianceVector;
    }

    void setTrajectoryDuration(double duration) {
        _trajectoryDuration = duration;
        updateActionDuration();
    }

    void setControlSequenceSteps(int n) {
        _n = n;
        updateActionDuration();
    }

    void resetToInitialState(const shared_ptr<MujocoHelper> mujocoHelper) {
        mujocoHelper->resetSimulation();
    }

protected:
    // Virtual Methods

    virtual bool isGoalAchieved(State) = 0;

    virtual CostSequence cost(StateSequence, ControlSequence) = 0;

    virtual ControlSequence createCandidateControlSequence() = 0;

    // Utilities

    void resetToInitialState() {
        resetToInitialState(_mujocoHelper);
    }

    double formatDurationToSeconds(std::chrono::high_resolution_clock::time_point start,
                                   std::chrono::high_resolution_clock::time_point end) {
        int milliseconds = chrono::duration_cast<chrono::milliseconds>(end - start).count();
        return milliseconds / 1000.0;
    }

    void updateActionDuration() {
        _actionDuration = _trajectoryDuration / _n;
    }

    void setMuJoCoTo(State &state, const shared_ptr<MujocoHelper> &mujocoHelper) {
        mujocoHelper->restoreFrom(state);
        mujocoHelper->forward();
    }

    void setMuJoCoTo(State &state) {
        setMuJoCoTo(state, _mujocoHelper);
    }

    State getCurrentStateFromMujoco(const shared_ptr<MujocoHelper> &mujocoHelper) {
        return mujocoHelper->getState();
    }

    void createLocalMujocoHelpers(const shared_ptr<MujocoHelper> &mujocoHelper) {
        _mujocoHelpers.clear();
        _mujocoHelpers.shrink_to_fit();

        for (int k = 0; k < _numberOfNoisyTrajectoryRollouts; k++) {
            _mujocoHelpers.push_back(make_shared<MujocoHelper>(mujocoHelper.get()));
        }
    }

    void createLocalMujocoHelpers() {
        createLocalMujocoHelpers(_mujocoHelper);
    }

    // Trajectory Update

    static ControlSequence greedyUpdate(vector<tuple<StateSequence, ControlSequence, CostSequence>> rollouts) {
        ControlSequence minControlSequence = get<1>(rollouts[0]);
        double minCost = get<2>(rollouts[0]).sum();

        for (unsigned int i = 1; i < rollouts.size(); i++) {
            double currentCost = get<2>(rollouts[i]).sum();
            if (currentCost < minCost) {
                minCost = currentCost;
                minControlSequence = get<1>(rollouts[i]);
            }
        }

        return minControlSequence;
    }

    ControlSequence probabilisticUpdate(ControlSequence controlSequence,
                                        vector<tuple<StateSequence, ControlSequence, CostSequence>> rollouts) {
        unsigned int n = get<2>(rollouts[0]).size();
        const double h = 10;

        ControlSequence newControlSequence(_n);

        for (unsigned int i = 0; i < n; ++i) {
            double maxCost = std::numeric_limits<double>::min();
            double minCost = std::numeric_limits<double>::max();

            for (auto rollout : rollouts) {
                // k
                CostSequence costSequence = get<2>(rollout);
                double currentCost = costSequence.getCost(i);
                if (currentCost < minCost)
                    minCost = currentCost;
                if (currentCost > maxCost)
                    maxCost = currentCost;
            }

            double denom = maxCost - minCost;
            if (denom < 1e-8)
                denom = 1e-8;

            double pSum = 0.0;
            for (auto rollout : rollouts) {
                CostSequence costSequence = get<2>(rollout);
                double currentCost = costSequence.getCost(i);
                double prob = exp(-h * (currentCost - minCost) / denom);
                pSum += prob;
            }

            double noiseLinearX = 0.0;
            double noiseLinearY = 0.0;
            double noiseAngularZ = 0.0;
            double noiseGripperDOFValue = 0.0;

            double linearX = controlSequence.getControl(i).getLinearX();
            double linearY = controlSequence.getControl(i).getLinearY();
            double angularZ = controlSequence.getControl(i).getAngularZ();
            double gripperDOFValue = controlSequence.getControl(i).getGripperDOF();

            for (auto rollout : rollouts) {
                CostSequence costSequence = get<2>(rollout);
                double currentCost = costSequence.getCost(i);
                double prob = exp(-h * (currentCost - minCost) / denom);
                prob = prob / pSum;
                ControlSequence controlSeq = get<1>(rollout);
                noiseLinearX += prob * controlSeq.getControl(i).getLinearXNoise();
                noiseLinearY += prob * controlSeq.getControl(i).getLinearYNoise();
                noiseAngularZ += prob * controlSeq.getControl(i).getAngularZNoise();

                if (_samplingVarianceVector->size() >= 3) {
                    noiseGripperDOFValue += prob * controlSeq.getControl(i).getGripperDOFNoise();

                    // make sure that the noise + the previous value won't make
                    // an invalid dof value.
                    if (gripperDOFValue + noiseGripperDOFValue > 255.0 ||
                        gripperDOFValue + noiseGripperDOFValue < 0.0) {
                        noiseGripperDOFValue = 0.0;
                    }
                }
            }

            Control control(linearX, linearY, angularZ, gripperDOFValue);
            control.setLinearXnoise(noiseLinearX);
            control.setLinearYnoise(noiseLinearY);
            control.setAngularZnoise(noiseAngularZ);
            control.setGripperDOFnoise(noiseGripperDOFValue);

            newControlSequence.addControl(control);
        }

        return newControlSequence;
    }

    ControlSequence updateTrajectory(ControlSequence controlSequence,
                                     const vector<tuple<StateSequence, ControlSequence, CostSequence>> &rollouts) {
        if (_USE_PROBABILISTIC_UPDATE)
            return probabilisticUpdate(std::move(controlSequence), rollouts);

        return greedyUpdate(rollouts);
    }

    // Noisy Trajectory Generation

    ControlSequence createNoisyTrajectoryFrom(ControlSequence controlSequence) {
        ControlSequence noisyControlSequence(_n);

        for (unsigned int j = 0; j < controlSequence.size(); ++j) {
            Control previousValue = controlSequence.getControl(j);

            double linearX = previousValue.getLinearX();
            double linearY = previousValue.getLinearY();
            double angularZ = previousValue.getAngularZ();
            double gripperDOF = previousValue.getGripperDOF();
            double linearXnoise = 0.0;
            double linearYnoise = 0.0;
            double angularZnoise = 0.0;
            double gripperDOFnoise = 0.0;

            std::random_device rd;
            std::mt19937 gen(rd());


            vector<double> gripperDOFs = {0.0, 100, 150, 255.0};
            std::random_device dev;
            std::mt19937 rng(dev());
            std::uniform_int_distribution<std::mt19937::result_type> dist6(0, gripperDOFs.size() - 1);

            for (unsigned int i = 0; i < _samplingVarianceVector->size(); ++i) {
                std::normal_distribution<double> d(0.0, _samplingVarianceVector->at(i));
                double noise = d(gen);

                if (i == 0) {
                    linearXnoise = noise;
                } else if (i == 1) {
                    linearYnoise = noise;
                } else if (i == 2) {
                    angularZnoise = noise;
                } else if (i == 3) { // Gripper DOF
                    double randomGripperDOF = gripperDOFs[dist6(rng)];
                    if (gripperDOF >= randomGripperDOF)
                        gripperDOFnoise = -(gripperDOF - randomGripperDOF);
                    else
                        gripperDOFnoise = (randomGripperDOF - gripperDOF);
                } else {
                    throw std::invalid_argument(
                            "You are attempting to sample a noise for more DOF than the DOFs of the system.");
                }
            }

            Control noisyControl(linearX, linearY, angularZ, gripperDOF);
            noisyControl.setLinearXnoise(linearXnoise);
            noisyControl.setLinearYnoise(linearYnoise);
            noisyControl.setAngularZnoise(angularZnoise);
            noisyControl.setGripperDOFnoise(gripperDOFnoise);
            noisyControlSequence.addControl(noisyControl);
        }

        return noisyControlSequence;
    }

    tuple<StateSequence, ControlSequence, CostSequence>
    generateNoisyTrajectoryThreaded(ControlSequence &controlSequence, int k) {
        auto mujocoHelper = _mujocoHelpers[k];
        resetToInitialState(mujocoHelper);
        auto noisyControlSequence = createNoisyTrajectoryFrom(controlSequence);
        auto rollout = trajectoryRollout(noisyControlSequence, mujocoHelper);
        auto stateSequence = get<0>(rollout);
        auto costSequence = get<1>(rollout);

        auto fullRollout = make_tuple(stateSequence, noisyControlSequence, costSequence);
        return fullRollout;
    }

    vector<tuple<StateSequence, ControlSequence, CostSequence>>
    createNoisyTrajectories(ControlSequence controlSequence) {

#pragma omp declare reduction (merge : std::vector<tuple<StateSequence, ControlSequence, CostSequence>> : omp_out.insert(omp_out.end(), omp_in.begin(), omp_in.end()))
        vector<tuple<StateSequence, ControlSequence, CostSequence>> rollouts;
#pragma omp parallel for reduction(merge: rollouts)
        for (int k = 0; k < _numberOfNoisyTrajectoryRollouts; ++k) {
            rollouts.push_back(generateNoisyTrajectoryThreaded(controlSequence, k));
        }


        return rollouts;

    }

    tuple<StateSequence, CostSequence> trajectoryRollout(const ControlSequence &controls) {
        return trajectoryRollout(controls, _mujocoHelper);
    }

    tuple<StateSequence, CostSequence>
    trajectoryRollout(const ControlSequence &controls, const shared_ptr<MujocoHelper> &mujocoHelper) {
        State currentState = _initialState;
        setMuJoCoTo(currentState, mujocoHelper);

        StateSequence stateSequence(_n);
        double steps = _actionDuration / mujocoHelper->getTimeStep();
        for (Control currentControl : controls.getControls()) {
            if(currentControl.isNull()) {
                continue;
            }

            vector<double> initialGripperJointValues = mujocoHelper->getCurrentJointValuesForGripper();
            vector<double> finalGripperJointValues = mujocoHelper->getJointValuesForGripper(
                    currentControl.getGripperDOF()); // Fully-closed


            vector<double> diffGripperJointValues = finalGripperJointValues;
            for (unsigned int i = 0; i < initialGripperJointValues.size(); ++i) {
                diffGripperJointValues[i] -= initialGripperJointValues[i];
            }

            for (int step = 0; step < steps; ++step) {
                mujocoHelper->setRobotVelocity(currentControl.getLinearX(), currentControl.getLinearY(),
                                               currentControl.getAngularZ());

                // Set gripper DOF
                vector<double> stepGripperJointValues = initialGripperJointValues;
                for (unsigned int i = 0; i < stepGripperJointValues.size(); ++i) {
                    stepGripperJointValues[i] += diffGripperJointValues[i] * (step / steps);
                }

                mujocoHelper->setGripperJointValues(stepGripperJointValues);
                mujocoHelper->step();
            }

            currentState = getCurrentStateFromMujoco(mujocoHelper);
            stateSequence.addState(currentState);
        }

        CostSequence costSequence = cost(stateSequence, controls);
        return make_tuple(stateSequence, costSequence);
    }

    shared_ptr<MujocoHelper> _mujocoHelper;
    State _initialState;
    int _numberOfNoisyTrajectoryRollouts = 8;
    int _maxIterations = 100;
    double _costThreshold = 100.0;
    double _trajectoryDuration = 10.0;
    int _n = 50;
    double _actionDuration = 1.0;
    std::vector<double> *_samplingVarianceVector = nullptr;
    std::vector<string> *_movableObstacleNames = nullptr;
    std::vector<string> *_staticObstacleNames = nullptr;
    bool _USE_PROBABILISTIC_UPDATE = true;
    Result _result = NullResult(0.0);
    bool _isOptimizing = false;

    // Local mujoco helpers to be used in threads when sampling.
    std::vector<shared_ptr<MujocoHelper>> _mujocoHelpers;
};

class InteractiveOptimizerBase : public OptimizerBase {
public:
    InteractiveOptimizerBase(const shared_ptr<MujocoHelper> &mujocoHelper) : OptimizerBase(mujocoHelper),
                                                                             _controlSequence((unsigned int) 0),
                                                                             _stateSequence((unsigned int) 0) {
    }

    Result optimize() override {
        _isOptimizing = true;
        auto optimizationStart = std::chrono::high_resolution_clock::now();

        _optimizationTimeInSeconds = 0.0;
        if (!_shouldReplan) {
            if (!_samplingVarianceVector) {
                throw std::runtime_error("The variance vector parameter was not set. Quitting");
            }

            // This is an initial candidate assuming to be the best so far.
            auto initialControlSequenceAndCost = getInitialControlSequenceAndItsCost();
            _controlSequence = get<0>(initialControlSequenceAndCost);
            _stateSequence = get<1>(initialControlSequenceAndCost);
            _bestCost = get<2>(initialControlSequenceAndCost);

            if (_controlSequence.size() == 0) {
                printf("\033[0;31mNot able to generate a valid, initial control sequence. Optimisation aborted.\033[0m\n");
                return NullResult(0);
            }
        } else {
            printf("Warm-starting!\n");

            State finalCurrentState = _stateSequence.getState(_n - 1);
            setMuJoCoTo(finalCurrentState);

            // Current position of goal object.
            double goalObjectX = _mujocoHelper->getBodyXpos("object_3");
            double goalObjectY = _mujocoHelper->getBodyYpos("object_3");

            // Find the vector of the end-effector to the goal object.
            auto endEffectorPosition = _mujocoHelper->getSitePosition("ee_point_1");
            double endEffectorX = endEffectorPosition[0];
            double endEffectorY = endEffectorPosition[1];
            double eeToGoalX = goalObjectX - endEffectorX;
            double eeToGoalY = goalObjectY - endEffectorY;

            vector<double> goalVector = {eeToGoalX, eeToGoalY};
            vector<double> unitVector = _mujocoHelper->unitVectorOf(goalVector);
            unitVector[0] *= 0.04;
            unitVector[1] *= 0.04;
            eeToGoalX -= unitVector[0];
            eeToGoalY -= unitVector[1];

            // Now convert end-effector point into robot point.
            Eigen::MatrixXf desiredEndEffectorTransform = Eigen::MatrixXf::Identity(4, 4);
            desiredEndEffectorTransform(0, 3) = eeToGoalX;
            desiredEndEffectorTransform(1, 3) = eeToGoalY;
            Eigen::MatrixXf endEffectorInRobot = _mujocoHelper->getEndEffectorInRobotTransform();
            Eigen::MatrixXf robotInEndEffector = endEffectorInRobot.inverse();
            Eigen::MatrixXf desiredRobotTransform = desiredEndEffectorTransform * robotInEndEffector;

            // Calculate direction vector
            auto site2 = _mujocoHelper->getSitePosition("ee_point_0");
            double site2_x = site2[0];
            double site2_y = site2[1];
            double eeVectorX = endEffectorX - site2_x;
            double eeVectorY = endEffectorY - site2_y;
            vector<double> directionVector = {eeVectorX, eeVectorY};

            double x1 = directionVector[0];
            double y1 = directionVector[1];

            auto robotPosition = _mujocoHelper->getSitePosition("ee_point_0");
            double robotX = robotPosition[0];
            double robotY = robotPosition[1];
            double eeToGoalX2 = goalObjectX - robotX;
            double eeToGoalY2 = goalObjectY - robotY;
            vector<double> robotVector = {eeToGoalX2, eeToGoalY2};

            double x2 = robotVector[0];
            double y2 = robotVector[1];

            auto dot = x1 * x2 + y1 * y2;
            auto det = x1 * y2 - y1 * x2;
            double angle = 0.3 * atan2(det, dot);//  # atan2(y, x) or atan2(sin, cos)

            int stepsLeft = 0;
            vector<int> stepsToUpdate;
            for (unsigned int i = 0; i < _controlSequence.size(); ++i) {
                if (_controlSequence.getControl(i).isNull()) {
                    stepsLeft++;
                    stepsToUpdate.push_back(i);
                }
            }

            double linearX = eeToGoalX / stepsLeft;
            double linearY = eeToGoalY / stepsLeft;
            double angularZ = angle / stepsLeft;

            for (int i = 0; i < stepsLeft; ++i) {
                _controlSequence.getPointerToControl(stepsToUpdate[i])->setLinearX(linearX);
                _controlSequence.getPointerToControl(stepsToUpdate[i])->setLinearY(linearY);
                _controlSequence.getPointerToControl(stepsToUpdate[i])->setAngularZ(angularZ);
            }

            _mujocoHelper->resetSimulation();

            _shouldReplan = false;
        }


        Result result(false, 0.0, _controlSequence);
        _result = result;

        int iterations = 0;
        auto optimizationFinish = std::chrono::high_resolution_clock::now();
        _optimizationTimeInSeconds += formatDurationToSeconds(optimizationStart, optimizationFinish);
        optimizationStart = std::chrono::high_resolution_clock::now();

        bool secondaryConditionNotMet;
        if (_isAdaptive)
            secondaryConditionNotMet = true;
        else
            secondaryConditionNotMet = _optimizationTimeInSeconds < _maxTimeOutForHelpLimit;

        bool costThresholdNotReached = _bestCost > _costThreshold;
        bool totalTimeLimitNotReached = _optimizationTimeInSeconds < _maxOptimizationTimeSeconds;

        vector<double> costSequence = {_bestCost};
        int consecutiveNotReducingCost = 0;
        int consecutiveLocalMinima = 0;
        double previousCost = numeric_limits<double>::min();

        while (costThresholdNotReached && totalTimeLimitNotReached && secondaryConditionNotMet) {
            iterations++;

            resetToInitialState();

            auto rollouts = createNoisyTrajectories(_controlSequence);

            resetToInitialState();

            ControlSequence currentControlSequence = updateTrajectory(_controlSequence, rollouts);

            auto currentRollout = trajectoryRollout(currentControlSequence);
            auto currentCost = get<1>(currentRollout).sum();

            printf("%d. Prior Cost: %f | Updated Cost: %f\n", iterations, _bestCost, currentCost);
            costSequence.push_back(currentCost);

            if (currentCost < _bestCost) {
                _controlSequence = currentControlSequence;
                _stateSequence = get<0>(currentRollout);
                _bestCost = currentCost;

                Result result2(false, 0.0, _controlSequence);
                _result = result2;
                consecutiveNotReducingCost = 0;
                consecutiveLocalMinima = 0;
            } else {
                consecutiveNotReducingCost++;
            }

            if (abs(previousCost - currentCost) < 50.0 || currentCost - _bestCost > 1000.0) {
                consecutiveLocalMinima++;
            } else
                consecutiveLocalMinima = 0;

            costThresholdNotReached = _bestCost > _costThreshold;
            optimizationFinish = std::chrono::high_resolution_clock::now();
            _optimizationTimeInSeconds += formatDurationToSeconds(optimizationStart, optimizationFinish);
            optimizationStart = std::chrono::high_resolution_clock::now();
            totalTimeLimitNotReached = _optimizationTimeInSeconds < _maxOptimizationTimeSeconds;

            if (_isAdaptive) {
                if (_isAdaptiveLocalMinima) {
                    if (consecutiveLocalMinima >= 2) {
                        printf("Adaptively (local-minima) timed-out.\n");
                        secondaryConditionNotMet = false;
                    }
                } else {
                    if (consecutiveNotReducingCost >= _numberOfConsecutiveNonDecreases) {
                        printf("Adaptively timed-out.\n");
                        secondaryConditionNotMet = false;
                    }
                }
            } else
                secondaryConditionNotMet = _optimizationTimeInSeconds < _maxTimeOutForHelpLimit;

            previousCost = currentCost;
        }

        resetToInitialState();

        auto bestStateSequence = get<0>(trajectoryRollout(_controlSequence));
        State finalState = bestStateSequence.getState(bestStateSequence.size() - 1);
        _tempState = finalState;

        optimizationFinish = std::chrono::high_resolution_clock::now();
        _optimizationTimeInSeconds += formatDurationToSeconds(optimizationStart, optimizationFinish);
        printf("Optimization time: %f\n", _optimizationTimeInSeconds);

        bool reachedGoal = false;
        if (_bestCost <= _costThreshold) {
            if (_subGoalSet) { // We were optimizing for a sub-goal problem.
                reachedGoal = isSubGoalAchieved(finalState);
                _reachedSubGoal = reachedGoal;
                if (reachedGoal) {
                    _subGoalSet = false;
                }
            } else {
                reachedGoal = isPrimaryGoalAchieved(finalState);
                _reachedPrimaryGoal = reachedGoal;
            }
        }

        Result finalResult(reachedGoal, _optimizationTimeInSeconds, _controlSequence);
        finalResult.setCostSequence(costSequence);
        _result = finalResult;
        _isOptimizing = false;
        return finalResult;
    }

    // MPPI algorithm to follow the optimized _controlSequence trajectory already optimized from this.optimize().
    Control followAlreadyOptimizedTrajectory(string actionType) {
        _mujocoHelper->resetSimulation();

        if (!_samplingVarianceVector) {
            throw std::runtime_error("The variance vector parameter was not set. Quiting");
        }

        if (_controlSequence.size() == 0) {
            // This is an initial candidate assuming to be the best so far.
            auto initialControlSequenceAndCost = getInitialControlSequenceAndItsCost();
            _controlSequence = get<0>(initialControlSequenceAndCost);
            _stateSequence = get<1>(initialControlSequenceAndCost);
            _bestCost = get<2>(initialControlSequenceAndCost);

            if (_controlSequence.size() == 0) {
                printf("\033[0;31mNot able to generate a valid, initial control sequence. Optimisation aborted.\033[0m\n");
                return {0.0, 0.0, 0.0, 0.0};
            }
        }

        State currentState = _initialState;
        _tempState = currentState;

        if (actionType == "push") { // We were optimizing for a sub-goal problem.
            _reachedSubGoal = isSubGoalAchieved(currentState);
            if (_reachedSubGoal) {
                printf("Sub-goal achieved in followAlreadyOptimizedTrajectory, sending zero vel!\n");
                return {0.0, 0.0, 0.0, 0.0};
            }
        } else {
            _reachedPrimaryGoal = isPrimaryGoalAchieved(currentState);
            if (_reachedPrimaryGoal) {
                return {0.0, 0.0, 0.0, 0.0};
            }
        }

        auto currentRollout = trajectoryRollout(_controlSequence);
        _bestCost = get<1>(currentRollout).sum();

        // Check if we can reach the goal state
        auto stateSequence = get<0>(currentRollout);
        State finalState = stateSequence.getState(stateSequence.size() - 1);

        bool reachedGoal;
        if (actionType == "push") { // We were optimizing for a sub-goal problem.
            reachedGoal = isSubGoalAchieved(finalState);
            _reachedSubGoal = reachedGoal;
        } else {
            reachedGoal = isPrimaryGoalAchieved(finalState);
            _reachedPrimaryGoal = reachedGoal;
        }

        if (reachedGoal) {
            auto firstControl = _controlSequence.getControl(0);
            _controlSequence = removeFirstStepAndExpandTrajectoryByOneStep(_controlSequence);
            return firstControl;
        } else {
            printf("Sorry, you should replan! The cost is: %f\n", _bestCost);
            if (actionType == "push")
                _subGoalSet = true;
            _shouldReplan = true;
        }

        return {0, 0, 0, 0};
    }

    void updateIfWeReachedTheGoal(string actionType) {
        State currentState = _mujocoHelper->getLatestSavedState();

        if (actionType == "push") { // We were optimizing for a sub-goal problem.
            _reachedSubGoal = isSubGoalAchieved(currentState);
            if (!_reachedSubGoal) {
                _subGoalSet = true;
            }
        } else {
            _reachedPrimaryGoal = isPrimaryGoalAchieved(currentState);
        }
    }

    void clearSubGoal() {
        _subGoalSet = false;
    }

    // Getters

    bool shouldReplan() {
        return _shouldReplan;
    }

    bool reachedPrimaryGoal() {
        return _reachedPrimaryGoal;
    }

    bool reachedSubGoal() {
        return _reachedSubGoal;
    }

    ControlSequence getControlSequence() {
        return _controlSequence;
    }

    // Setters

    void setMaxOptimizationTime(double maxTimeInSeconds) {
        _maxOptimizationTimeSeconds = maxTimeInSeconds;
    }

    void setSubGoal(double x, double y) {
        _subGoalDesiredX = x;
        _subGoalDesiredY = y;
        _reachedSubGoal = false;
        _reachedPrimaryGoal = false;
        _subGoalSet = true;
    }

    void updateMujocoHelpers(const shared_ptr<MujocoHelper> &mujocoHelper) {
        // Create a MuJoCo Helper for each of the K noisy trajectories (for palatalization).
        createLocalMujocoHelpers(mujocoHelper);
        _mujocoHelper = make_shared<MujocoHelper>(mujocoHelper.get());
    }

    void setAdaptiveLocalMinima(bool isAdaptiveLocalMinima) {
        _isAdaptiveLocalMinima = isAdaptiveLocalMinima;
    }

    void setAdaptive(bool isAdaptive) {
        _isAdaptive = isAdaptive;
    }

    void setTimeOutForHelpLimit(double timeLimit) {
        _maxTimeOutForHelpLimit = timeLimit;
    }

    void setAdaptiveNumberOfConesecutiveNonDecreases(int number) {
        _numberOfConsecutiveNonDecreases = number;
    }

protected:
    virtual bool isPrimaryGoalAchieved(State) = 0;

    virtual bool isSubGoalAchieved(State) = 0;

    ControlSequence removeFirstStepAndExpandTrajectoryByOneStep(ControlSequence controlSequence) {
        ControlSequence newControlSequence(_n);
        for (int i = 1; i < _n; ++i) {
            Control currentControl = controlSequence.getControl(i);
            newControlSequence.addControl(currentControl);
        }

        Control velocitiesWithOpenGripper(0.0, 0.0, 0.0, 0.0);
        newControlSequence.addControl(velocitiesWithOpenGripper);
        return newControlSequence;
    }

    double _maxTimeOutForHelpLimit = 0.0;
    int _numberOfConsecutiveNonDecreases;
    ControlSequence _controlSequence;
    StateSequence _stateSequence;
    double _bestCost = std::numeric_limits<double>::max();
    bool _shouldReplan = false;
    bool _reachedPrimaryGoal = false;
    bool _reachedSubGoal = false;
    bool _subGoalSet = false;
    bool _isAdaptive = false;
    bool _isAdaptiveLocalMinima = false;
    double _subGoalDesiredX;
    double _subGoalDesiredY;
    double _maxOptimizationTimeSeconds = std::numeric_limits<double>::max();
    State _tempState;
    double _optimizationTimeInSeconds = 0.0;
    bool _print = false;
};

#endif
