#include "robot_config.h"


// Vertical Tracking Wheel
pros::Rotation rotation_sensor(-11);

pros::Imu imu(1); // CHANGE THIS!!!

pros::MotorGroup left_motors({20, -19, -15}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({-18, 17, 16}, pros::MotorGearset::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.5, // 11.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450rpm (im pretty sure)
                              5 // horizontal drift (omni + traction wheel mix)
);

// tracking wheel configuration
lemlib::TrackingWheel vertical_tracking_wheel(&rotation_sensor, lemlib::Omniwheel::NEW_275, .375);
// vertical wheel, 2.75" diameter, -.25" offset from tracking center

// odometry sensors configuration
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // no second vertical wheel
                            nullptr, // no horizontal tracking wheel
                            nullptr, // no second horizontal wheel
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(4.25,   // kP
                                              0,   // kI
                                              1,   // kD
                                              3,   // anti windup
                                              1,   // small error range
                                              100, // small error range timeout
                                              3,   // large error range
                                              500, // large error range timeout
                                              0  // slew - START HIGH
);


// angular PID controller - TUNED
lemlib::ControllerSettings angular_controller(.863   , // kP
                                              0,   // kI
                                              0.235,  // kD
                                              3,   // anti windup - ENABLE
                                              1,   // small error range, in degrees - ENABLE
                                              100, // small error range timeout - ENABLE
                                              3,   // large error range, in degrees - ENABLE
                                              500, // large error range timeout - ENABLE
                                              0    // slew
);

// Add these BEFORE the chassis declaration
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.05);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.05);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve,
                        &steer_curve
);

// --------------------- Motors ---------------------

pros::Motor Intake(INTAKE_PORT, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Motor Outtake(OUTTAKE_PORT, pros::MotorGears::blue, pros::MotorUnits::degrees);

// --------------------- Sensors ---------------------
pros::adi::DigitalOut Descore('A');
pros::adi::DigitalOut Unloader('C');
pros::adi::DigitalOut MidScoring('B');

// --------------------- Controller ---------------------
pros::Controller master(pros::E_CONTROLLER_MASTER);



void initializeRobot() {
    // Non-blocking calibration - fixes "Run" mode hang
    chassis.calibrate(false);
    
    // Set brake modes
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);   // Prevents drifting, smooth control
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);  // Prevents drifting, smooth control
    Outtake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);        // Holds position, prevents backdriving
    Intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);         // Prevents backdriving when stopped
    
    Outtake.set_reversed(false);
    Intake.set_reversed(true);
    MidScoring.set_value(false);

}