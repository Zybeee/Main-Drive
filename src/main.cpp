#include "main.h"
#include "autonomous.h"
#include "curves.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "subsystems/intake.h"
#include "subsystems/outtake.h"
#include "subsystems/pneumatics.h"
#include <string>


void initialize() { initializeRobot(); }

void disabled() {}

void competition_initialize() {}

void autonomous() {
  skills_auton();
  // leftAuton();
  // rightAuton();
  // leftAuton_descore();
}

// Small deadband to prevent drift (applies to values close to 0)
int applyDeadband(int value, int threshold = 8) {
  return (abs(value) < threshold) ? 0 : value;
}

void opcontrol() {
  IntakeControl intake;
  OuttakeControl outtake;
  PneumaticControl pneumatics;

  // Initialize descore piston to extended position
  Descore.set_value(true);

  // ── Joystick curve selection ──
  // Change the default here, or press DOWN arrow during driving to cycle.
  CurveType activeCurve = CurveType::SQUARED;
  bool curveButtonPressed = false;

  // Tracking for warnings (don't spam alerts)
  uint32_t lastTempCheck = 0;
  uint32_t lastBatteryCheck = 0;
  bool lowBatteryWarned = false;

  while (true) {

    // Display intake current draw + active curve on brain screen
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::fill_rect(0, 0, 480, 40);
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::print(
        pros::E_TEXT_LARGE, 10, 10, "Intake mA: %d  |  Curve: %s",
        Intake.get_current_draw(), getCurveName(activeCurve).c_str());

    // ── Cycle curve with DOWN arrow ──
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      if (!curveButtonPressed) {
        activeCurve = nextCurve(activeCurve);
        master.print(2, 0, "Curve: %s       ",
                     getCurveName(activeCurve).c_str());
        curveButtonPressed = true;
      }
    } else {
      curveButtonPressed = false;
    }

    // Tank Drive with deadband + curve
    int left = applyCurve(
        applyDeadband(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)),
        activeCurve);
    int right = applyCurve(
        applyDeadband(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)),
        activeCurve);
    left_motors.move(left);
    right_motors.move(right);

    // Update subsystems
    outtake.update(intake);
    intake.update(outtake);
    pneumatics.update();

    // === MOTOR TEMPERATURE MONITORING (every 2 seconds) ===
    if (pros::millis() - lastTempCheck > 2000) {
      lastTempCheck = pros::millis();

      // Check all motor temperatures (overheat starts at 55°C)
      double maxTemp = 0;
      std::string hotMotor = "";

      // Check drive motors (get_temperature returns vector for groups)
      auto leftTemps = left_motors.get_temperature_all();
      auto rightTemps = right_motors.get_temperature_all();

      for (double temp : leftTemps) {
        if (temp > maxTemp) {
          maxTemp = temp;
          hotMotor = "L-Drive";
        }
      }
      for (double temp : rightTemps) {
        if (temp > maxTemp) {
          maxTemp = temp;
          hotMotor = "R-Drive";
        }
      }
      if (Intake.get_temperature() > maxTemp) {
        maxTemp = Intake.get_temperature();
        hotMotor = "Intake";
      }
      if (Outtake.get_temperature() > maxTemp) {
        maxTemp = Outtake.get_temperature();
        hotMotor = "Outtake";
      }

      // Warn at 50°C (before power reduction kicks in at 55°C)
      if (maxTemp >= 50) {
        master.print(0, 0, "HOT: %s %.0fC   ", hotMotor.c_str(), maxTemp);
      }
    }

    // === LOW BATTERY WARNING (10%) ===
    if (pros::millis() - lastBatteryCheck > 5000) {
      lastBatteryCheck = pros::millis();

      int batteryLevel = pros::battery::get_capacity();
      if (batteryLevel <= 10 && !lowBatteryWarned) {
        master.rumble("---"); // Long rumble pattern
        master.print(1, 0, "LOW BATTERY: %d%%", batteryLevel);
        lowBatteryWarned = true;
      }
    }

    pros::delay(20);
  }
}