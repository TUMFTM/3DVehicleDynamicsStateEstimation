// Copyright 2023 Marcel Weinmann
#include <gtest/gtest.h>
#include <chrono>
#include "tum_types_cpp/common.hpp"
#include "state_estimation_cpp/state_estimation.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

/**
 * @brief Update the State Estimaion State Machine
 * 
 * OK:        State Estimation functions as expected
 * WARN:      The qualitiy of the state estimation prediction is degraded (no emergency)
 * ERROR:     An important sensor signal is missing (execute a soft emergency stop on EKF)
 * STALE:     All safety critical sensors are missing (execute a hard emergency stop)
 * 
 * Now:
 *                        |  OK  | STALE |  WARN  | STALE | ERROR | STALE | ERROR | STALE
 * ---------------------------------------------------------------------------------------
 * any valid_loc_x        |   x  |   x   |    x   |   x   |       |       |       |
 * ---------------------------------------------------------------------------------------
 * any valid_lin_vel_x    |   x  |   x   |        |       |   x   |   x   |       |
 * ---------------------------------------------------------------------------------------
 * any valid_imu_x        |   x  |       |    x   |       |   x   |       |   x   |
 */

TEST(ExecutionUnitTests, StateMachineInit) {
    /**
     * expected output        |  OK  |
     * -------------------------------
     * any valid_loc_x        |   x  |
     * -------------------------------
     * any valid_lin_vel_x    |   x  |
     * -------------------------------
     * any valid_imu_x        |   x  |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::OK);
}

TEST(ExecutionUnitTests, StateMachineInitError_1) {
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::STALE);
}

TEST(ExecutionUnitTests, StateMachineInitError_2) {
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::STALE);
}

TEST(ExecutionUnitTests, StateMachineIMUTimeout) {
    /**
     * expected output        |  STALE  |
     * ----------------------------------
     * any valid_loc_x        |    x    |
     * ----------------------------------
     * any valid_lin_vel_x    |    x    |
     * ----------------------------------
     * any valid_imu_x        |         |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // IMU Timeout
    StateEstimation.set_imu_timeout(0);
    StateEstimation.set_imu_timeout(1);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::STALE);
}

TEST(ExecutionUnitTests, StateMachineOneIMUNoError) {
    /**
     * expected output        |    OK   |
     * ----------------------------------
     * any valid_loc_x        |    x    |
     * ----------------------------------
     * any valid_lin_vel_x    |    x    |
     * ----------------------------------
     * one valid_imu_x        |    >1    |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set three imus valid
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 2);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // IMU Timeout
    StateEstimation.set_imu_timeout(0);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::OK);
}

TEST(ExecutionUnitTests, StateMachineOneBackupIMUError) {
    /**
     * expected output        |  ERROR  |
     * ----------------------------------
     * any valid_loc_x        |    x    |
     * ----------------------------------
     * any valid_lin_vel_x    |    x    |
     * ----------------------------------
     * one valid_imu_x        |    1    |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // get the param handler
    auto param_handler = StateEstimation.get_param_handler();
    param_handler->set_parameter_value("P_VDC_MinValidIMUs", 1);

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set two imus and the backup imu valid
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 3);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // IMU Timeout
    StateEstimation.set_imu_timeout(0);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::ERROR);
}

TEST(ExecutionUnitTests, StateMachineOneBackupIMUStale) {
    /**
     * expected output        |  STALE  |
     * ----------------------------------
     * any valid_loc_x        |    x    |
     * ----------------------------------
     * any valid_lin_vel_x    |    x    |
     * ----------------------------------
     * no valid_imu_x         |    0    |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // get the param handler
    auto param_handler = StateEstimation.get_param_handler();
    param_handler->set_parameter_value("P_VDC_MinValidIMUs", 1);

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set two imus and the backup imu valid
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 3);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // IMU Timeout
    StateEstimation.set_imu_timeout(0);
    StateEstimation.set_imu_timeout(3);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::STALE);
}

TEST(ExecutionUnitTests, StateMachineOneBackupIMULocFailStale) {
    /**
     * expected output        |  STALE  |
     * ----------------------------------
     * any valid_loc_x        |         |
     * ----------------------------------
     * any valid_lin_vel_x    |    x    |
     * ----------------------------------
     * no valid_imu_x         |    1    |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // get the param handler
    auto param_handler = StateEstimation.get_param_handler();
    param_handler->set_parameter_value("P_VDC_MinValidIMUs", 1);

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set two imus and the backup imu valid
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 3);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // IMU Timeout
    StateEstimation.set_imu_timeout(0);

    // Localization Error
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::ERROR, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::ERROR, 0);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::STALE);
}

TEST(ExecutionUnitTests, StateMachineOneBackupIMUVelFailStale) {
    /**
     * expected output        |  STALE  |
     * ----------------------------------
     * any valid_loc_x        |    x    |
     * ----------------------------------
     * any valid_lin_vel_x    |         |
     * ----------------------------------
     * no valid_imu_x         |    1    |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // get the param handler
    auto param_handler = StateEstimation.get_param_handler();
    param_handler->set_parameter_value("P_VDC_MinValidIMUs", 1);

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set two imus and the backup imu valid
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 3);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // IMU Timeout
    StateEstimation.set_imu_timeout(0);

    // Linear Velocity Error
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::ERROR);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::STALE);
}

TEST(ExecutionUnitTests, StateMachineOneBackupIMULocVelFailStale) {
    /**
     * expected output        |  STALE  |
     * ----------------------------------
     * any valid_loc_x        |         |
     * ----------------------------------
     * any valid_lin_vel_x    |         |
     * ----------------------------------
     * no valid_imu_x         |    1    |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // get the param handler
    auto param_handler = StateEstimation.get_param_handler();
    param_handler->set_parameter_value("P_VDC_MinValidIMUs", 1);

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set two imus and the backup imu valid
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 3);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // IMU Timeout
    StateEstimation.set_imu_timeout(0);

    // Linear Velocity and Loc Error
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::ERROR);
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::ERROR, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::ERROR, 0);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::STALE);
}

TEST(ExecutionUnitTests, StateMachineOneIMUError) {
    /**
     * expected output         |  ERROR  |
     * -----------------------------------
     * any valid_loc_x         |    x    |
     * -----------------------------------
     * any valid_lin_vel_x     |    x    |
     * -----------------------------------
     * one  valid_imu_x        |    1    |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set two imus valid
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // IMU Timeout
    StateEstimation.set_imu_timeout(0);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::ERROR);
}

TEST(ExecutionUnitTests, StateMachineWheelspeedTimeout) {
    /**
     * expected output        |  ERROR  |
     * ---------------------------------
     * any valid_loc_x        |    x    |
     * ---------------------------------
     * any valid_lin_vel_x    |         |
     * ---------------------------------
     * any valid_imu_x        |    x    |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // Wheelspeed Timeout
    StateEstimation.set_wheelspeeds_timeout();

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::ERROR);
}

TEST(ExecutionUnitTests, StateMachineSteeringTimeout) {
    /**
     * expected output        |  ERROR  |
     * ---------------------------------
     * any valid_loc_x        |    x    |
     * ---------------------------------
     * any valid_lin_vel_x    |         |
     * ---------------------------------
     * any valid_imu_x        |    x    |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D>
        StateEstimation("single-track-model");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);
    StateEstimation.set_input_steering_angle_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // Steering Angle Timeout
    StateEstimation.set_steering_angle_timeout();

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::ERROR);
}

TEST(ExecutionUnitTests, StateMachineIMUandWheelspeedTimeout) {
    /**
     * expected output        |  STALE  |
     * ----------------------------------
     * any valid_loc_x        |    x    |
     * ----------------------------------
     * any valid_lin_vel_x    |         |
     * ----------------------------------
     * any valid_imu_x        |         |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // Wheelspeed and IMU timeout
    StateEstimation.set_imu_timeout(0);
    StateEstimation.set_imu_timeout(1);
    StateEstimation.set_wheelspeeds_timeout();

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::STALE);
}

TEST(ExecutionUnitTests, StateMachineLocWarn) {
    /**
     * expected output        |  ERROR  |
     * ----------------------------------
     * any valid_loc_x        |         |
     * ----------------------------------
     * any valid_lin_vel_x    |    x    |
     * ----------------------------------
     * any valid_imu_x        |    x    |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // Localization warn
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::WARN, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::WARN, 0);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::OK);
}

TEST(ExecutionUnitTests, StateMachineLocError) {
    /**
     * expected output        |  ERROR  |
     * ----------------------------------
     * any valid_loc_x        |         |
     * ----------------------------------
     * any valid_lin_vel_x    |    x    |
     * ----------------------------------
     * any valid_imu_x        |    x    |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // Localization error
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::ERROR, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::ERROR, 0);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::ERROR);
}

TEST(ExecutionUnitTests, StateMachineLocErrorandIMUTimeout) {
    /**
     * expected output        |  STALE  |
     * ----------------------------------
     * any valid_loc_x        |         |
     * ----------------------------------
     * any valid_lin_vel_x    |    x    |
     * ----------------------------------
     * any valid_imu_x        |         |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // Localization error and imu timeout
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::ERROR, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::ERROR, 0);
    StateEstimation.set_imu_timeout(0);
    StateEstimation.set_imu_timeout(1);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::STALE);
}

TEST(ExecutionUnitTests, StateMachineLocErrorandWheelspeedTimeout) {
    /**
     * expected output        |  ERROR  |
     * ----------------------------------
     * any valid_loc_x        |         |
     * ----------------------------------
     * any valid_lin_vel_x    |         |
     * ----------------------------------
     * any valid_imu_x        |    x    |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // Localization error and wheelspeed timeout
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::ERROR, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::ERROR, 0);
    StateEstimation.set_wheelspeeds_timeout();

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::ERROR);
}

TEST(ExecutionUnitTests, StateMachineFullFailure) {
    /**
     * expected output        |  STALE  |
     * ----------------------------------
     * any valid_loc_x        |         |
     * ----------------------------------
     * any valid_lin_vel_x    |         |
     * ----------------------------------
     * any valid_imu_x        |         |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // Full sensor failure all modalities are invalid
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::ERROR, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::ERROR, 0);
    StateEstimation.set_wheelspeeds_timeout();
    StateEstimation.set_imu_timeout(0);
    StateEstimation.set_imu_timeout(1);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::STALE);
}

TEST(ExecutionUnitTests, StateMachineSTALEBuffer) {
    /**
     * expected output        |  STALE  |
     * ----------------------------------
     * any valid_loc_x        |         |
     * ----------------------------------
     * any valid_lin_vel_x    |         |
     * ----------------------------------
     * any valid_imu_x        |         |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // Full sensor failure all modalities are invalid
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::ERROR, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::ERROR, 0);
    StateEstimation.set_wheelspeeds_timeout();
    StateEstimation.set_imu_timeout(0);
    StateEstimation.set_imu_timeout(1);

    // All sensor modalities are back but STALE should be buffered
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::STALE);
}

TEST(ExecutionUnitTests, StateMachineERRORBuffer) {
    /**
     * expected output        |  ERROR  |
     * ----------------------------------
     * any valid_loc_x        |         |
     * ----------------------------------
     * any valid_lin_vel_x    |    x    |
     * ----------------------------------
     * any valid_imu_x        |    x    |
     */
    // Create an instance of tam::core::state::StateEstimation
    tam::core::state::StateEstimation<tam::core::state::EKF_3D> StateEstimation("non-holonomic");

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry;
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed;
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    StateEstimation.set_initial_state();

    // Localization error
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::ERROR, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::ERROR, 0);

    // All sensor modalities are back but ERROR should be buffered
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);

    // compare the state estimation output
    EXPECT_EQ(StateEstimation.get_status(), tam::types::ErrorLvl::ERROR);
}
