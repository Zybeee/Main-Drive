#pragma once
#include "main.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/distance.hpp"

// Motor ports
constexpr int INTAKE_PORT = 13;
constexpr int OUTTAKE_PORT = 12;

// Declare all hardware (using extern so they're defined once in .cpp)
extern pros::Rotation rotation_sensor;
extern pros::Imu imu;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;

extern lemlib::Drivetrain drivetrain;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::OdomSensors sensors;
extern lemlib::Chassis chassis;

extern pros::Motor Intake;
extern pros::Motor Outtake;

extern pros::adi::DigitalOut Descore;
extern pros::adi::DigitalOut Unloader;
extern pros::adi::DigitalOut MidScoring;

extern pros::Controller master;

extern pros::Optical optical_sensor;

extern pros::Distance back_sensor_left;
extern pros::Distance back_sensor_right;
extern pros::Distance left_sensor;
extern pros::Distance right_sensor;


// Initialization function
void initializeRobot();
