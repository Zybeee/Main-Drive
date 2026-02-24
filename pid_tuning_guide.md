# PID Tuning Guide — LemLib

This guide covers how to tune the **2 PID controllers** in LemLib to get accurate, consistent autonomous movement.

---

## Overview: The 2 PID Controllers

LemLib uses two main PID controllers to handle all movement.

| Controller      | Variables in `robot_config.cpp`   | What It Controls                                                         |
| --------------- | --------------------------------- | ------------------------------------------------------------------------ |
| **Lateral PID** | `lateral_controller` (kP, kI, kD) | Straight-line driving (how far to go)                                    |
| **Angular PID** | `angular_controller` (kP, kI, kD) | Turning in place AND keeping straight while driving (heading correction) |

All values are in `src/robot_config.cpp`. **Tune Lateral first, then Angular.**

---

## How PID Works (Quick Refresher)

- **kP (Proportional)** — "How hard to push based on how far away I am." Higher = more aggressive, but too high = overshoot/oscillation.
- **kI (Integral)** — "Am I stuck? Push harder over time." Accumulates error over time.
- **kD (Derivative)** — "Slow down when approaching the target." Dampens oscillation. Higher = smoother stops but too high = sluggish response.

> [!TIP]
> For VEX robots, **kI is usually 0**. LemLib handles most steady-state error through its exit conditions. Only add kI if your robot consistently stops ~1 inch short of target.

---

## Setup

### Before You Start

- **Battery Voltage**: Always tune your PID with a mostly charged battery. A dying battery has less voltage, which can make a perfectly tuned P-value suddenly look too sluggish.
- **Robot Weight**: Do your final tuning when the robot is fully built. If you tune the drivebase bare, and then add 5 lbs of intakes and lifts, your `kP` and `kD` will be completely wrong because the robot's momentum has changed.
- **Field Conditions**: Always tune on the foam field tiles, not a hard floor. The friction of the foam drastically changes how much `kD` (braking) you need.

### Steps

1. Put your test code in `autonomous()` in `src/main.cpp` (or `src/autonomous.cpp`).
2. Run the autonomous, observe, adjust PID values in `src/robot_config.cpp`, rebuild, repeat.

> **Tip:** Make sure to reset odometry `chassis.setPose(0, 0, 0);` at the top of your test auton so it starts fresh!

---

## Part 1: Lateral PID (Distance)

### What This Controls

When you call `chassis.moveToPoint(0, 24, 3000)`, this PID calculates how much voltage to send to the motors based on how far the robot has traveled vs. the target distance.

### Tuning Process

#### Step 1: Tune kP (set kI and kD to 0 first)

Test code:

```cpp
void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 3000); // Drive 24 inches forward
}
```

- **Robot barely moves** → kP too low, increase by 1-2
- **Robot doesn't reach target** → increase kP
- **Robot overshoots and comes back** → decrease kP
- **Robot oscillates back and forth** → kP is way too high, cut it in half
- **Goal:** Robot gets close to 24 inches and should overshoot slightly (this means kP is strong enough).

#### Step 2: Add kD to stop oscillation

- If robot still overshoots → increase kD
- If robot is sluggish/crawls or takes too long to arrive → decrease kD
- **Goal:** Robot arrives at 24 inches cleanly with minimal oscillation.

#### Step 3: Exit Conditions (Instead of kI)

Keep `kI` at 0. If it stops short or takes too long to finish the move, tweak your `small_error_range`, `large_error_range`, and their timeouts in the `lateral_controller` setup.

### Validation Tests (ALL must pass)

Don't just test one distance! Run all of these and adjust until they're all accurate:

```cpp
void autonomous() {
    // Test 1: Short distance forward
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 12, 3000);
}
```

```cpp
void autonomous() {
    // Test 2: Medium distance forward
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 3000);
}
```

```cpp
void autonomous() {
    // Test 3: Long distance forward
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 48, 4000);
}
```

```cpp
void autonomous() {
    // Test 4: Backward
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -24, 3000, {.forwards = false});
}
```

```cpp
void autonomous() {
    // Test 5: Forward then backward (tests stopping accuracy)
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 3000);
    pros::delay(500);
    chassis.moveToPoint(0, 0, 3000, {.forwards = false});
}
```

**Acceptable accuracy: ±1 inch** for all tests. If short distances work but long ones don't (or vice versa), your kP is probably off.

---

## Part 2: Angular PID (Turn & Heading)

### What This Controls

When you call `chassis.turnToHeading(90, 2000)`, this PID calculates motor voltage based on the difference between current heading and target heading.
**It also handles Heading Correction** when you use `moveToPoint` to make sure the robot drives straight without drifting left or right!

> [!NOTE]
> Angular kP values in LemLib are typically **much smaller** than lateral kP. Your current value of ~0.863 is in a reasonable range.

### Tuning Process

#### Step 1: Tune kP (set kI and kD to 0)

Test code:

```cpp
void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 2000); // Turn to 90 degrees
}
```

- **Robot doesn't reach 90°** → increase kP
- **Robot overshoots past 90° and comes back** → decrease kP
- **Robot oscillates around 90°** → kP too high
- **Goal:** Robot gets close to 90° and will wobble/overshoot perfectly on center.

#### Step 2: Add kD

- If robot overshoots → increase kD
- If robot takes forever to settle → decrease kD
- **Goal:** Robot snaps to 90° cleanly.

### Validation Tests (ALL must pass)

This is the critical part — **a PID that only works at 90° is useless.** Test all of these:

```cpp
void autonomous() {
    // Test 1: 90° right
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 2000);
}
```

```cpp
void autonomous() {
    // Test 2: 90° left
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(-90, 2000); // or 270
}
```

```cpp
void autonomous() {
    // Test 3: 180° (half turn)
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(180, 2000);
}
```

```cpp
void autonomous() {
    // Test 4: 270° (three-quarter turn)
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(270, 3000);
}
```

```cpp
void autonomous() {
    // Test 5: 360° (full rotation)
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(360, 3000);
}
```

```cpp
void autonomous() {
    // Test 6: Small angle (hardest to tune)
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(30, 2000);
}
```

```cpp
void autonomous() {
    // Test 7: Sequential turns (tests heading tracking)
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 2000);
    pros::delay(300);
    chassis.turnToHeading(0, 2000);
    pros::delay(300);
    chassis.turnToHeading(-90, 2000);
    pros::delay(300);
    chassis.turnToHeading(0, 2000);
}
```

```cpp
void autonomous() {
    // Test 8: 45° — common in actual autons
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(45, 2000);
    pros::delay(300);
    chassis.turnToHeading(135, 2000);
    pros::delay(300);
    chassis.turnToHeading(0, 2000);
}
```

**Acceptable accuracy: ±2°** for all tests. If 90° works but 270° doesn't, your kD may be too low (the robot coasts past on longer turns because it picks up too much speed).

### Common Issues

| Symptom                                | Fix                                                       |
| -------------------------------------- | --------------------------------------------------------- |
| Accurate at 90° but overshoots at 270° | Increase kD — robot needs more braking for longer turns   |
| Undershoots small angles (30°, 45°)    | Increase kP slightly — not enough initial push            |
| Oscillates on all angles               | kP too high, decrease by 20%                              |
| Takes forever to settle                | kD too high, decrease by 20%                              |
| Different overshoot CW vs CCW          | Tune for the average to minimize error in both directions |

---

## Part 3: Heading Correction Validation

LemLib uses the Angular PID to automatically correct heading while driving straight. If your lateral and angular PIDs are solid, your heading correction should be too.

### Validation Tests

```cpp
void autonomous() {
    // Test 1: Long drive forward (should be perfectly straight)
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 48, 5000);
}
```

```cpp
void autonomous() {
    // Test 2: Drive, turn, drive (tests chained drift)
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 3000);
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(24, 24, 3000);
}
```

```cpp
void autonomous() {
    // Test 3: Square test (the ultimate test)
    // Robot should end up close to where it started
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 3000);
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(24, 24, 3000);
    chassis.turnToHeading(180, 2000);
    chassis.moveToPoint(24, 0, 3000);
    chassis.turnToHeading(270, 2000);
    chassis.moveToPoint(0, 0, 3000);
    chassis.turnToHeading(0, 2000);
}
```

**The square test is the gold standard.** If your robot ends up close to where it started (within ~2 inches), your PIDs are well-tuned.

---

## Exit Conditions — Often the Real Problem

Sometimes the robot _looks_ badly tuned but the PID is fine — it's just exiting too early or too late.

Your current exit conditions (in `src/robot_config.cpp`):

```
Lateral:  small error = 1 inch / 100ms,  large error = 3 inches / 500ms
Angular:  small error = 1 degree / 100ms,  large error = 3 degrees / 500ms
```

| Symptom                                                 | Likely cause                                       | Fix                          |
| ------------------------------------------------------- | -------------------------------------------------- | ---------------------------- |
| Robot stops short every time                            | Large error timeout too short                      | Increase large error timeout |
| Robot sits at target for too long before moving on      | Small error timeout too long                       | Decrease small error timeout |
| Robot oscillates at the end and takes forever to settle | PID is fine but error keeps crossing the threshold | Widen small error range      |

---

## The Asymmetric Drivetrain Problem

### Why CW and CCW turns aren't equal

This is **extremely common** and is caused by:

1. **Friction differences** — one side of the drivetrain has more friction (tighter gears, bent shaft, motor wear)
2. **Motor differences** — motors aren't identical, one side is slightly stronger
3. **Weight distribution** — robot is heavier on one side

### What you can do about it

- **Mechanical fixes (best solution):** Check for bent shafts, ensure gears mesh evenly, verify wheels spin freely.
- **Tune for the "average":** Since LemLib uses one angular PID for both directions, **tune so both CW and CCW are acceptable** rather than perfect for one direction. If 90° CW is perfect but 90° CCW overshoots by 3°, increase kD slightly. The result: ±1.5° error in both directions instead of 0° in one and 3° in the other.
- **Use the IMU:** The IMU already compensates for a lot of drivetrain asymmetry because it measures **actual heading**. Make sure your IMU is mounted near the center, firmly attached, and properly calibrated.

---

## Final Checklist

- [ ] Lateral PID: Robot drives 12", 24", 48" accurately (±1")
- [ ] Lateral PID: Robot drives backward accurately
- [ ] Angular PID: Robot turns to 30°, 45°, 90°, 180°, 270°, 360° accurately (±2°)
- [ ] Angular PID: Sequential turns (back and forth) are accurate
- [ ] Heading Correction: Long drives (48") are perfectly straight, no curving
- [ ] Heading Correction: Square test — robot returns to start position (±2")

---

## Appendix: Ziegler-Nichols Tuning Method (Semi-Automatic)

If you prefer a mathematical approach over guess-and-check, you can use the **Ziegler-Nichols** method to calculate your baseline `kP`, `kI`, and `kD` values.

### How to use it:

1. Set `kI` and `kD` to `0`.
2. Slowly increase `kP` until the robot starts oscillating (wobbling) over the target **indefinitely and at a constant rate** (the wobbles don't stop, and they don't get smaller or larger).
3. Record this exact `kP` value. This is your **Ultimate Gain ($K_u$)**.
4. Time how long it takes for one full wobble (one full oscillation crest-to-crest) in seconds. This is your **Oscillation Period ($T_u$)**.

### The Formulas:

Once you have $K_u$ and $T_u$, calculate your PID values:

- **kP** = $0.6 \times K_u$
- **kI** = $1.2 \times (K_u / T_u)$ _(Keep at 0 for LemLib regardless)_
- **kD** = $0.075 \times K_u \times T_u$

_Note: This method often produces a very "aggressive" tune. You may still need to manually decrease `kP` or increase `kD` slightly to smooth out the movement._
