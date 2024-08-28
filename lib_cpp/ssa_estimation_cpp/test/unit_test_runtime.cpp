// Copyright 2024 Sven Goblirsch
#include <gtest/gtest.h>
#include <chrono>
#include "ssa_estimation_cpp/ssa_estimation.hpp"
#include "ssa_estimation_constants/UKF_STM.hpp"
#include "tum_types_cpp/common.hpp"

TEST(ExecutionUnitTests, ExecutionTimeUKF) {
    // Create an instance of tam::core::ssa::SSAEstimation
    tam::core::ssa::SSAEstimation<tam::core::ssa::UKF_STM> SSAEstimation =
        tam::core::ssa::SSAEstimation<tam::core::ssa::UKF_STM>();
    int64_t execution_time = 0;

    // set the sensor status to initialize the state machine
    SSAEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK);
    SSAEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    SSAEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    SSAEstimation.set_input_steering_angle_status(tam::types::ErrorLvl::OK);
    SSAEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry = tam::types::control::Odometry();
    SSAEstimation.set_input_vehicle_orientation(input_odometry);
    tam::types::common::DataPerWheel<double> input_wheelspeed =
        tam::types::common::DataPerWheel<double>();
    SSAEstimation.set_input_wheelspeeds(input_wheelspeed);
    SSAEstimation.set_input_steering_angle(0.0);
    tam::types::control::AccelerationwithCovariances input_acceleration =
        tam::types::control::AccelerationwithCovariances();
    SSAEstimation.set_input_acceleration(input_acceleration, 0);
    SSAEstimation.set_input_acceleration(input_acceleration, 1);

    for (int i = 0; i < 1000; i++) {
        // start time of the function execution
        auto start = std::chrono::high_resolution_clock::now();

        // set the sensor status to initialize the state machine
        SSAEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK);
        SSAEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
        SSAEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
        SSAEstimation.set_input_steering_angle_status(tam::types::ErrorLvl::OK);
        SSAEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

        // set a random odometry and wheelspeed to allow the initial state to be set
        tam::types::control::Odometry input_odometry = tam::types::control::Odometry();
        SSAEstimation.set_input_vehicle_orientation(input_odometry);

        tam::types::common::DataPerWheel<double> input_wheelspeed =
            tam::types::common::DataPerWheel<double>();
        SSAEstimation.set_input_wheelspeeds(input_wheelspeed);

        SSAEstimation.set_input_steering_angle(0.0);
        tam::types::control::AccelerationwithCovariances input_acceleration =
            tam::types::control::AccelerationwithCovariances();
        SSAEstimation.set_input_acceleration(input_acceleration, 0);
        SSAEstimation.set_input_acceleration(input_acceleration, 1);

        // one step of the state estimation
        SSAEstimation.step();

        // get the state estimation output
        SSAEstimation.get_status();
        SSAEstimation.get_odometry();
        SSAEstimation.get_state_machine_debug_output();
        SSAEstimation.get_kalman_filter_debug_output();

        // end time
        auto end = std::chrono::high_resolution_clock::now();

        // Calculate the execution time in microseconds
        execution_time +=
            std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }

    // run the test (divide by 1000 to get the mean)
    // one step souldn't take more than 200 microseconds
    EXPECT_LE(execution_time / 1e3, 200);
}