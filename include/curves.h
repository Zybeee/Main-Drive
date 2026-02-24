#pragma once
#include <cmath>
#include <string>

/**
 * Joystick curve types for drive control.
 *
 * Switch between these during testing to find what feels best.
 * Each curve maps raw joystick input (-127 to 127) to motor output (-127 to
 * 127).
 */
enum class CurveType {
  LINEAR,      // 1:1 mapping — raw and direct
  SQUARED,     // x² — smooth low-end, popular default
  CUBED,       // x³ — very smooth low-end, snappy top-end
  EXPONENTIAL, // e^(k*x) — tunable aggressiveness
  S_CURVE,     // Sigmoid — smooth start AND end
  PIECEWISE,   // Two-zone linear — simple and predictable
  PLATEAU      // Custom 3-zone curve with a precise "cruising speed" flat spot
};

/**
 * Apply the selected joystick curve to a raw input value.
 *
 * @param input    Raw joystick value (-127 to 127)
 * @param curve    Which curve type to apply
 * @param param    Tuning parameter (used by EXPONENTIAL and PIECEWISE):
 *                   - EXPONENTIAL: aggressiveness (default 1.5, range ~0.5-3.0)
 *                   - PIECEWISE: breakpoint as fraction (default 0.3, range
 * 0.1-0.5)
 * @return         Curved output value (-127 to 127)
 */
int applyCurve(int input, CurveType curve, double param = 0.0);

/**
 * Get a human-readable name for the curve type (for on-screen display).
 */
std::string getCurveName(CurveType curve);

/**
 * Cycle to the next curve type (wraps around).
 * Useful for binding to a button to quickly swap curves during testing.
 */
CurveType nextCurve(CurveType current);
