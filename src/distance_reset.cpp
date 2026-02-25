// ============================================================================
// DISTANCE SENSOR POSITION RESET FUNCTIONS (LemLib port of updated RW functions)
//
// GLOBAL SENSOR DECLARATIONS (add to your main.cpp or robot-config):
//   pros::Distance back_sensor_left(PORT);
//   pros::Distance back_sensor_right(PORT);
//   pros::Distance left_sensor(PORT);
//   pros::Distance right_sensor(PORT);
//
// GLOBAL OFFSET CONSTANTS (measure carefully in inches):
//   const double back_sensor_left_offset  = ?; // tracking center to left back sensor face
//   const double back_sensor_right_offset = ?; // tracking center to right back sensor face
//   const double back_sensor_spacing      = ?; // horizontal distance between two back sensor faces
//   const double left_sensor_offset       = ?; // tracking center to left sensor face
//   const double right_sensor_offset      = ?; // tracking center to right sensor face
//   const double field_half_size          = 72.0; // standard VEX field
//
// REQUIREMENTS:
//   - PROS + LemLib
//   - lemlib::Chassis object declared globally (named `chassis`)
// ============================================================================

#include "lemlib/api.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include <cmath>

extern lemlib::Chassis chassis;

// ============================================================================
// resetPositionAndHeadingBack
// Resets BOTH position AND heading using two back-facing distance sensors.
//
// How it works:
//   1. Reads both back sensors (left and right)
//   2. Calculates the angle to the wall from the difference in readings
//   3. Calculates the true perpendicular distance using cos(angle)
//   4. Determines which wall we're facing using current heading
//   5. Resets the appropriate axis (X or Y) to the corrected position
//   6. Resets heading using the calculated wall angle
//
// IMPORTANT: Call this when the back of the robot is facing a wall.
// Works best when roughly perpendicular — dual sensors correct for small angles.
// ============================================================================
void resetPositionAndHeadingBack(pros::Distance& back_left, pros::Distance& back_right,
                                  double sensor_spacing,
                                  double left_offset,   double right_offset,
                                  double field_half = 72.0) {

    double d_left  = back_left.get()  / 25.4; // mm to inches
    double d_right = back_right.get() / 25.4;

    // Validate readings
    if (d_left < 0 || d_left > 200 || d_right < 0 || d_right > 200) {
        printf("Invalid back sensor readings: L=%.1f R=%.1f\n", d_left, d_right);
        return;
    }

    // Calculate angle to wall from the two sensor readings
    // Positive angle = robot is rotated clockwise from perpendicular
    double angle_to_wall_rad = atan2(d_right - d_left, sensor_spacing);
    double angle_to_wall_deg = angle_to_wall_rad * 180.0 / M_PI;

    // Calculate corrected perpendicular distance
    double avg_offset   = (left_offset + right_offset) / 2.0;
    double avg_reading  = (d_left + d_right) / 2.0;
    double corrected_distance = avg_reading * cos(angle_to_wall_rad) + avg_offset;

    // Determine which wall the BACK of the robot is facing
    // Back of robot = heading + 180°
    lemlib::Pose pose = chassis.getPose();
    double back_heading_deg = pose.theta + 180.0;
    int headingDeg = ((int)back_heading_deg % 360 + 360) % 360;

    bool   resettingX = false;
    double wallSign   = 1.0;
    double expected_perpendicular_heading = 0.0;

    if (headingDeg >= 315 || headingDeg <= 45) {
        // Back faces top wall → reset Y (positive side)
        resettingX = false;
        wallSign   = 1.0;
        expected_perpendicular_heading = 180.0;
    }
    else if (headingDeg > 45 && headingDeg <= 135) {
        // Back faces right wall → reset X (positive side)
        resettingX = true;
        wallSign   = 1.0;
        expected_perpendicular_heading = 270.0;
    }
    else if (headingDeg > 135 && headingDeg <= 225) {
        // Back faces bottom wall → reset Y (negative side)
        resettingX = false;
        wallSign   = -1.0;
        expected_perpendicular_heading = 0.0;
    }
    else {
        // Back faces left wall → reset X (negative side)
        resettingX = true;
        wallSign   = -1.0;
        expected_perpendicular_heading = 90.0;
    }

    // Calculate corrected position
    double actualPos = wallSign * (field_half - corrected_distance);

    // Calculate corrected heading
    // CW rotation moves right rear sensor closer → atan2(d_right - d_left) is negative for CW
    // Subtracting the angle correctly converts to global heading
    double corrected_heading = expected_perpendicular_heading - angle_to_wall_deg;

    // Normalize heading to 0-360
    corrected_heading = fmod(corrected_heading + 360, 360);

    // Apply corrected pose — only update the relevant axis, keep the other
    double new_x = resettingX ? actualPos : pose.x;
    double new_y = resettingX ? pose.y    : actualPos;
    chassis.setPose(new_x, new_y, corrected_heading);

    printf("Reset: pos=%.1f hdg=%.1f (wall_angle=%.1f)\n",
           actualPos, corrected_heading, angle_to_wall_deg);
}

// ============================================================================
// resetPositionLeft
// Resets position using a single left-facing distance sensor.
// Uses trig correction (IMU heading) for more accurate results
// when not perfectly perpendicular to the wall.
// Only resets the appropriate axis (X or Y) based on which wall the sensor faces.
//
// IMPORTANT: Call this when the left side of the robot is facing a wall.
// ============================================================================
void resetPositionLeft(pros::Distance& sensor, double sensor_offset,
                       double field_half = 72.0) {

    double sensorReading = sensor.get() / 25.4;

    if (sensorReading < 0 || sensorReading > 200) {
        printf("Invalid left sensor reading: %.2f\n", sensorReading);
        return;
    }

    lemlib::Pose pose = chassis.getPose();

    // Left sensor direction = robot heading + 270° (pointing left)
    double sensor_heading_deg = pose.theta + 270.0;
    int headingDeg = ((int)sensor_heading_deg % 360 + 360) % 360;

    // Trig correction: find how far off perpendicular we are from the nearest wall
    double nearest_perpendicular = round(sensor_heading_deg / 90.0) * 90.0;
    double angle_off_deg = sensor_heading_deg - nearest_perpendicular;
    double angle_off_rad = angle_off_deg * M_PI / 180.0;

    // Correct the reading for the angle
    double corrected_distance = sensorReading * cos(angle_off_rad) + sensor_offset;

    // Determine which wall
    bool   resettingX = false;
    double wallSign   = 1.0;

    if      (headingDeg >= 315 || headingDeg <= 45)  { resettingX = false; wallSign =  1.0; } // Top wall
    else if (headingDeg > 45  && headingDeg <= 135)  { resettingX = true;  wallSign =  1.0; } // Right wall
    else if (headingDeg > 135 && headingDeg <= 225)  { resettingX = false; wallSign = -1.0; } // Bottom wall
    else                                              { resettingX = true;  wallSign = -1.0; } // Left wall

    double actualPos = wallSign * (field_half - corrected_distance);

    double new_x = resettingX ? actualPos : pose.x;
    double new_y = resettingX ? pose.y    : actualPos;
    chassis.setPose(new_x, new_y, pose.theta);
}

// ============================================================================
// resetPositionRight
// Resets position using a single right-facing distance sensor.
// Uses trig correction (IMU heading) for more accurate results
// when not perfectly perpendicular to the wall.
// Only resets the appropriate axis (X or Y) based on which wall the sensor faces.
//
// IMPORTANT: Call this when the right side of the robot is facing a wall.
// ============================================================================
void resetPositionRight(pros::Distance& sensor, double sensor_offset,
                        double field_half = 72.0) {

    double sensorReading = sensor.get() / 25.4;

    if (sensorReading < 0 || sensorReading > 200) {
        printf("Invalid right sensor reading: %.2f\n", sensorReading);
        return;
    }

    lemlib::Pose pose = chassis.getPose();

    // Right sensor direction = robot heading + 90°
    double sensor_heading_deg = pose.theta + 90.0;
    int headingDeg = ((int)sensor_heading_deg % 360 + 360) % 360;

    // Trig correction
    double nearest_perpendicular = round(sensor_heading_deg / 90.0) * 90.0;
    double angle_off_deg = sensor_heading_deg - nearest_perpendicular;
    double angle_off_rad = angle_off_deg * M_PI / 180.0;

    double corrected_distance = sensorReading * cos(angle_off_rad) + sensor_offset;

    // Determine which wall
    bool   resettingX = false;
    double wallSign   = 1.0;

    if      (headingDeg >= 315 || headingDeg <= 45)  { resettingX = false; wallSign =  1.0; } // Top wall
    else if (headingDeg > 45  && headingDeg <= 135)  { resettingX = true;  wallSign =  1.0; } // Right wall
    else if (headingDeg > 135 && headingDeg <= 225)  { resettingX = false; wallSign = -1.0; } // Bottom wall
    else                                              { resettingX = true;  wallSign = -1.0; } // Left wall

    double actualPos = wallSign * (field_half - corrected_distance);

    double new_x = resettingX ? actualPos : pose.x;
    double new_y = resettingX ? pose.y    : actualPos;
    chassis.setPose(new_x, new_y, pose.theta);
}

// ============================================================================
// driveUntilDistance
// Drives the robot until a distance sensor reads below a threshold, then stops.
// Useful for lining up before calling a position reset.
//
// - sensor:       distance sensor facing the wall you're driving toward
// - threshold_in: stop when sensor reads at or below this value (inches)
// - speed:        motor speed 0-127 (default 60)
// - forwards:     true = drive forward, false = drive backward
// - timeout_ms:   emergency stop time in milliseconds (default 3000)
//
// EXAMPLE:
//   driveUntilDistance(back_sensor_left, 3.0, 50, false, 3000);
//   resetPositionAndHeadingBack(...);
// ============================================================================
void driveUntilDistance(pros::Distance& sensor, double threshold_in,
                        int speed = 60, bool forwards = true, int timeout_ms = 3000) {

    int direction = forwards ? 1 : -1;
    int elapsed   = 0;

    chassis.tank(direction * speed, direction * speed, true);

    while (elapsed < timeout_ms) {
        // Only check distance if the sensor confirms an object is in range
        if (sensor.get() != PROS_ERR) {
            double reading = sensor.get() / 25.4;
            if (reading > 0 && reading <= threshold_in) {
                break;
            }
        }

        pros::delay(10);
        elapsed += 10;
    }

    // Stop with hold brake to prevent drift after stopping
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.tank(0, 0, true);
}
