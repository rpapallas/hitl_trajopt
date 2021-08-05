//  Copyright (C) 2019 Rafael Papallas and The University of Leeds
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//  Author: Rafael Papallas (rpapallas.com)

#include "../utils/MujocoGlobal.cpp"
#include "../src/discrete/StraightLineOptimizer.cpp"
#include "../utils/CommonFunctions.cpp"
#include "Execution.cpp"
#include <boost/optional.hpp>

const string EXPERIMENT_PATH =
        PROJECT_ROOT_PATH + "experiments/results/";
const string PLANNER_NAME = "OR_HITL";
vector<string> STATIC_OBSTACLE_NAMES = {"shelf"};
vector<string> MOVABLE_OBSTACLE_NAMES;
int NUMBER_OF_MOVABLE_OBJECTS;

bool MPPI_EXECUTION = true;

// ================================================================
// GENERAL PARAMETERS
// ================================================================
const double PLANE_HIGH_X = 0.58;
const double PLANE_LOW_X = 0.20;
const double PLANE_HIGH_Y = 0.38;
const double PLANE_LOW_Y = -0.38;
const double TABLE_Z = 0.18;

// ================================================================
// TRAJECTORY OPTIMISATION PARAMETERS FOR PUSHING
// ================================================================
string SCENE_NAME;
const string GOAL_OBJECT_NAME = "object_3";
int TRAJECTORY_ROLLOUTS = 15;
double COST_THRESHOLD = 70.0;
int TRAJECTORY_DURATION = 3;
int CONTROL_SEQUENCE_STEPS = 8;
vector<double> VARIANCE_VECTOR = {0.04, 0.04, 0.04};

// Parameters for experiments
bool ADAPTIVE = false;
bool ADAPTIVE_LOCAL_MINIMA = false;
int NUMBER_OF_CONSECUTIVE_NON_DECREASES = 5;
double TIMEOUT_FOR_HELP = 180.0;
double TOTAL_TIME_LIMIT = 180.0;

double INTERACTION_TIME = 0.0;
bool plannerIsIdle = false;
double totalIdleTime = 0.0;
boost::optional<std::chrono::high_resolution_clock::time_point> humanReturnedStartClock;
boost::optional<std::chrono::high_resolution_clock::time_point> humanReturnedWhilePlannerIdleStartClock;
std::chrono::high_resolution_clock::time_point plannerWaitingStartClock;

void quickRender() {
    render();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

void saveScreenshot(const string &extraMessage, int experimentId) {
    const string experimentNum = std::to_string(experimentId);
    std::string screenshotFileName =
            EXPERIMENT_PATH + "screenshots/experiment_" + experimentNum + "_" + extraMessage + ".tga";
    quickRender();
    saveScreenshotToFile(screenshotFileName, WINDOW_WIDTH, WINDOW_HEIGHT);
}

double formatDurationToSeconds(std::chrono::high_resolution_clock::time_point start,
                               std::chrono::high_resolution_clock::time_point end) {
    int milliseconds = chrono::duration_cast<chrono::milliseconds>(end - start).count();
    return milliseconds / 1000.0;
}

void window_focus_callback(GLFWwindow */*window*/, int windowIsFocused) {
    if (windowIsFocused) {
        humanReturnedStartClock = std::chrono::high_resolution_clock::now();
        if (plannerIsIdle) {
            humanReturnedWhilePlannerIdleStartClock = humanReturnedStartClock;
            totalIdleTime += formatDurationToSeconds(plannerWaitingStartClock, humanReturnedStartClock.get());
        } else {
            humanReturnedWhilePlannerIdleStartClock = boost::none;
        }
    } else {
        if (plannerIsIdle) {
            plannerWaitingStartClock = std::chrono::high_resolution_clock::now();
        }

        if (humanReturnedWhilePlannerIdleStartClock && plannerIsIdle) {
            auto now = std::chrono::high_resolution_clock::now();
            INTERACTION_TIME += formatDurationToSeconds(humanReturnedWhilePlannerIdleStartClock.get(), now);
        }

        humanReturnedStartClock = boost::none;
    }
}

int getNextId(const string &filename) {
    ifstream inputFile(EXPERIMENT_PATH + filename);
    int experimentNumber;
    inputFile >> experimentNumber;
    return experimentNumber;
}

int getNextActionId() {
    return getNextId("next_action_id.txt");
}

int getNextExperimentId() {
    return getNextId("next_experiment_id.txt");
}

void updateNextId(const string &filename, int id) {
    id++;
    std::ofstream outfile;
    outfile.open(EXPERIMENT_PATH + filename);
    outfile << id << "\n";
}

void updateNextExperimentId(int experimentId) {
    updateNextId("next_experiment_id.txt", experimentId);
}

void updateNextActionId(int actionId) {
    updateNextId("next_action_id.txt", actionId);
}

void writeAction(int experimentId, int actionId, string actionType, string outcome, double optimizationTime,
                 double interactionTime) {
    std::ofstream outfile;
    outfile.open(EXPERIMENT_PATH + "actions.txt", std::ios_base::app);

    outfile << experimentId
            << ", "
            << actionId
            << ", "
            << actionType
            << ", "
            << outcome
            << ", "
            << optimizationTime
            << ", "
            << interactionTime
            << "\n";
}

void writeExperiment(int experimentId, string outcome, string updateType) {
    std::ofstream outfile;
    outfile.open(EXPERIMENT_PATH + "experiments.txt", std::ios_base::app);

    string timeout;
    if (ADAPTIVE && ADAPTIVE_LOCAL_MINIMA)
        timeout = "adaptive_local_minima";
    else if (ADAPTIVE)
        timeout = "adaptive_" + to_string(NUMBER_OF_CONSECUTIVE_NON_DECREASES);
    else if (TIMEOUT_FOR_HELP == TOTAL_TIME_LIMIT)
        timeout = "no_timeout";
    else
        timeout = to_string(TIMEOUT_FOR_HELP);

    string plannerName = MPPI_EXECUTION ? "mppi_execution" : "nomppi_execution";

    outfile << experimentId
            << ", "
            << plannerName
            << ", "
            << SCENE_NAME
            << ", "
            << NUMBER_OF_MOVABLE_OBJECTS
            << ", "
            << outcome
            << ", "
            << TOTAL_TIME_LIMIT
            << ", "
            << timeout
            << ", "
            << TRAJECTORY_ROLLOUTS
            << ", "
            << COST_THRESHOLD
            << ", "
            << TRAJECTORY_DURATION
            << ", "
            << CONTROL_SEQUENCE_STEPS
            << ", "
            << VARIANCE_VECTOR[0]
            << ", "
            << VARIANCE_VECTOR[1]
            << ", "
            << VARIANCE_VECTOR[2]
            << ", "
            << updateType
            << "\n";
}

void writeCostSequence(int actionid, double cost) {
    std::ofstream outfile;
    outfile.open(EXPERIMENT_PATH + "cost_sequence.txt", std::ios_base::app);

    outfile << actionid
            << ", "
            << cost
            << "\n";
}

void executeControl(Control control, double controlDuration) {
    // Wait for control to finish.
    finishedExecution = false;

    executeRidgebackControlSimulationNoise(control, controlDuration, MOVABLE_OBSTACLE_NAMES);

    finishedExecution = true;
}

void moveBack() {
    finishedExecution = false;
    auto solution = moveRobotBackWithTrajectoryOptimization(MOVABLE_OBSTACLE_NAMES, STATIC_OBSTACLE_NAMES, PLANE_HIGH_X,
                                                            PLANE_LOW_X, PLANE_HIGH_Y, PLANE_LOW_Y, TABLE_Z);

    auto optimalControlSequence = get<0>(solution);
    auto actionDuration = get<1>(solution);

    double steps = actionDuration / globalMujocoHelper->getTimeStep();

    for (auto control : optimalControlSequence.getControls()) {
        for (int step = 0; step < steps; ++step) {
            globalMujocoHelper->setRobotVelocity(control.getLinearX(), control.getLinearY(), control.getAngularZ());
            globalMujocoHelper->step();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    finishedExecution = true;
}

void solve(bool saveResults) {
    StraightLineInteractiveOptimizer optimizer(globalMujocoHelper, PLANE_HIGH_X, PLANE_LOW_X, PLANE_HIGH_Y,
                                               PLANE_LOW_Y);
    optimizer.setNumberOfNoisyTrajectoryRollouts(TRAJECTORY_ROLLOUTS);
    optimizer.setTrajectoryDuration(TRAJECTORY_DURATION);
    optimizer.setSamplingVarianceVector(&VARIANCE_VECTOR);
    optimizer.setStaticObstacleNames(&STATIC_OBSTACLE_NAMES);
    optimizer.setMovableObstacleNames(&MOVABLE_OBSTACLE_NAMES);
    optimizer.setControlSequenceSteps(CONTROL_SEQUENCE_STEPS);
    optimizer.setCostThreshold(COST_THRESHOLD);
    optimizer.setGoalObjectName(GOAL_OBJECT_NAME);
    optimizer.setMaxOptimizationTime(TOTAL_TIME_LIMIT);
    optimizer.setTimeOutForHelpLimit(TIMEOUT_FOR_HELP);

    optimizer.setAdaptive(ADAPTIVE);
    optimizer.setAdaptiveLocalMinima(ADAPTIVE_LOCAL_MINIMA);
    optimizer.setAdaptiveNumberOfConesecutiveNonDecreases(NUMBER_OF_CONSECUTIVE_NON_DECREASES);

    State initialStartState = globalMujocoHelper->getState();
    optimizer.setInitialState(initialStartState);

    Result result = NullResult(false);

    int experimentId = getNextExperimentId();
    int actionId = getNextActionId() - 1;
    string actionType = "reach";
    double totalTime = 0.0;
    double totalInteractionTime = 0.0;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    bool replanning = false;

    bool failedDueExecution = false;

    while (!optimizer.reachedPrimaryGoal()) {
        double totalRunningTime = totalTime + totalInteractionTime;

        if (totalRunningTime >= TOTAL_TIME_LIMIT) {
            printf("Timed-out, quitting, %f\n", totalRunningTime);
            break;
        }

        globalMujocoHelper->resetSimulation();
        double remainingPlanningTime = TOTAL_TIME_LIMIT - totalRunningTime;
        optimizer.setMaxOptimizationTime(remainingPlanningTime);

        thread t1(&StraightLineInteractiveOptimizer::optimize, std::ref(optimizer));

        std::this_thread::sleep_for(std::chrono::seconds(1));
        while (optimizer.isOptimizing()) {
            finishedExecution = false;
            thread t2(executeSolutionFast, optimizer.getControlSequence(), optimizer.getActionDuration());

            while (!finishedExecution) {
                render();
            }

            t2.join();
            globalMujocoHelper->resetSimulation();
        }

        t1.join();

        result = optimizer.getResult();
        totalTime += result.getOptimizationTime();

        if (result.isSuccessful()) {
            globalMujocoHelper->resetSimulation();

            if (saveResults) {
                if (!replanning)
                    actionId++;
                writeAction(experimentId, actionId, actionType, "success_sim", result.getOptimizationTime(),
                            INTERACTION_TIME);

                vector<double> costSequence = result.getCostSequence();
                for (double cost : costSequence) {
                    writeCostSequence(actionId, cost);
                }
            }

            if (MPPI_EXECUTION) {
                Control control(0.0, 0.0, 0.0, 0.0);
                do {
                    State startState = globalMujocoHelper->getState();
                    globalMujocoHelper->saveState();
                    optimizer.setInitialState(startState);
                    optimizer.updateMujocoHelpers(globalMujocoHelper);

                    control = optimizer.followAlreadyOptimizedTrajectory(actionType);

                    if (control.isNotNull()) {
                        finishedExecution = false;

                        MPPI_EXECUTION = true;
                        thread t3(executeControl, control, optimizer.getActionDuration());

                        while (!finishedExecution) {
                            render();
                        }

                        t3.join();

                        startState = globalMujocoHelper->getState();
                        globalMujocoHelper->saveState();
                        optimizer.setInitialState(startState);
                        optimizer.updateMujocoHelpers(globalMujocoHelper);
                    }
                } while (control.isNotNull());
            } else {
                printf("executing without replanning.\n");
                globalMujocoHelper->resetSimulation();
                auto controlSequence = optimizer.getControlSequence();

                for (auto control : controlSequence.getControls()) {
                    finishedExecution = false;
                    thread t3(executeControl, control, optimizer.getActionDuration());

                    while (!finishedExecution) {
                        render();
                    }

                    t3.join();
                }

                State startState = globalMujocoHelper->getState();
                globalMujocoHelper->saveState();
                optimizer.setInitialState(startState);
                optimizer.updateMujocoHelpers(globalMujocoHelper);
                optimizer.updateIfWeReachedTheGoal(actionType);
            }

            if (actionType == "push")
                replanning = !optimizer.reachedSubGoal();
            else
                replanning = !optimizer.reachedPrimaryGoal();

            if (replanning)
                failedDueExecution = true;

            if (saveResults) {
                optimizer.updateMujocoHelpers(globalMujocoHelper);
                optimizer.updateIfWeReachedTheGoal(actionType);
                string outcome;

                if (actionType == "push")
                    outcome = optimizer.reachedSubGoal() ? "success_exec" : "failure_exec";
                else
                    outcome = optimizer.reachedPrimaryGoal() ? "success_exec" : "failure_exec";

                if (outcome == "success_exec")
                    replanning = false;

                writeAction(experimentId, actionId, actionType, outcome, result.getOptimizationTime(),
                            INTERACTION_TIME);
            }

            globalMujocoHelper->resetSimulation();

            if (!optimizer.reachedPrimaryGoal() && optimizer.reachedSubGoal() && !optimizer.shouldReplan()) {
                finishedExecution = false;
                thread t3(moveBack);

                while (!finishedExecution) {
                    render();
                }

                t3.join();

                // Changed to system happened since last time.
                globalMujocoHelper->saveState();
                State startState = globalMujocoHelper->getState();
                optimizer.setInitialState(startState);
                optimizer.updateMujocoHelpers(globalMujocoHelper);
                optimizer.setGoalObjectName(GOAL_OBJECT_NAME);
                actionType = "reach";
                INTERACTION_TIME = 0.0; // Here we are interested in per action interaction time, not overall interaction time.
            }
        } else {
            if (saveResults) {
                actionId++;
                writeAction(experimentId, actionId, actionType, "failure_sim", result.getOptimizationTime(),
                            INTERACTION_TIME);

                vector<double> costSequence = result.getCostSequence();
                for (double cost : costSequence) {
                    writeCostSequence(actionId, cost);
                }
            }
        }

        if (totalTime + totalInteractionTime >= TOTAL_TIME_LIMIT || optimizer.reachedPrimaryGoal()) {
            printf("Timed-out, quiting: %f seconds or reached primary goal: %d.\n", totalTime + totalInteractionTime,
                   optimizer.reachedPrimaryGoal());
            break;
        }

        // Ask human-input.
        if (!optimizer.isOptimizing() && !result.isSuccessful() && !optimizer.shouldReplan()) {
            // Human Input
            INTERACTION_TIME = 0.0; // Here we are interested in per action interaction time, not overall interaction time.

            plannerIsIdle = true;
            plannerWaitingStartClock = std::chrono::high_resolution_clock::now();

            string objectNameSelected = getObjectSelection(MOVABLE_OBSTACLE_NAMES);

            if (objectNameSelected == GOAL_OBJECT_NAME) {
                State startState = globalMujocoHelper->getState();
                optimizer.setInitialState(startState);
                optimizer.setGoalObjectName(GOAL_OBJECT_NAME);
                optimizer.clearSubGoal();
                actionType = "reach";
            } else {
                auto clickPosition = getPushPosition();
                double positionToPushObjectX = get<0>(clickPosition);
                double positionToPushObjectY = get<1>(clickPosition);

                // Make this a sub-goal.
                optimizer.setGoalObjectName(objectNameSelected);
                optimizer.setSubGoal(positionToPushObjectX, positionToPushObjectY);
                actionType = "push";
            }

            auto plannerWaitingFinishClock = std::chrono::high_resolution_clock::now();
            plannerIsIdle = false;

            // There was idle time
            if (humanReturnedStartClock && plannerWaitingStartClock < humanReturnedStartClock) {
                INTERACTION_TIME += formatDurationToSeconds(humanReturnedStartClock.get(),
                                                            plannerWaitingFinishClock);
                totalInteractionTime += INTERACTION_TIME;
            } else {
                INTERACTION_TIME += formatDurationToSeconds(plannerWaitingStartClock, plannerWaitingFinishClock);
                totalInteractionTime += INTERACTION_TIME;
            }
        }
    }

    // Write experiment result outcome.
    if (saveResults) {
        optimizer.updateMujocoHelpers(globalMujocoHelper);
        optimizer.updateIfWeReachedTheGoal(actionType);
        updateNextActionId(actionId);

        string failure = failedDueExecution ? "failure_exec" : "failure";
        string outcome = optimizer.reachedPrimaryGoal() ? "success" : failure;
        string updateType = optimizer.isProbabilisticUpdate() ? "probabilistic" : "greedy";
        writeExperiment(experimentId, outcome, updateType);
        updateNextExperimentId(experimentId);
        saveScreenshot("final_state", experimentId);
    }
}

int main(int argc, char **argv) {
    if (argc < 2 || (strcmp(argv[1], "--help") == 0)) {
        printf("Usage:\n");
        printf("rosrun   project_name   program_name   model_name   number_of_movable_objects\n");
        return 1;
    }

    mj_activate(MJ_KEY_PATH.c_str());

    SCENE_NAME = argv[1];
    NUMBER_OF_MOVABLE_OBJECTS = atoi(argv[2]);

    bool saveResult = false;
    if (argc > 2)
        saveResult = atoi(argv[3]);

    if (argc > 3)
        MPPI_EXECUTION = atoi(argv[4]);

    MOVABLE_OBSTACLE_NAMES = getObjectNamesFromNumberOfObjects(NUMBER_OF_MOVABLE_OBJECTS);

    initMujocoFrom(SCENE_PATH + SCENE_NAME);
    globalMujocoHelper->enableGripperDOF();
    globalMujocoHelper->setMovableObjectNames(MOVABLE_OBSTACLE_NAMES);
    globalMujocoHelper->forward();
    globalMujocoHelper->saveState();

    glfwShowWindow(window);
    glfwSetWindowFocusCallback(window, window_focus_callback);

    solve(saveResult);
}
