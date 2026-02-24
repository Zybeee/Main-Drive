# PID Tuning Guide for LemLib

## What kP, kI, kD Actually Do

| Constant | What it does | Too high | Too low |
|----------|-------------|----------|---------|
| **kP** | "How hard do I push toward the target?" — output is proportional to error | Overshoots, oscillates back and forth | Sluggish, never reaches target or takes forever |
| **kI** | "Am I stuck? Push harder over time" — accumulates error over time | Oscillates wildly, winds up | Robot stops slightly short of target (steady-state error) |
| **kD** | "Am I approaching too fast? Slow down" — reacts to rate of change | Jittery, shaky movement | Overshoots because nothing dampens the approach |

> [!TIP]
> For VEX robots, **kI is usually 0**. LemLib handles most steady-state error through its exit conditions. Only add kI if your robot consistently stops ~1 inch short of target.

---

## Tuning Order: Always kP → kD → kI

### Step 1: Tune Lateral (Driving Straight)

**Set kD = 0, kI = 0. Only tune kP first.**

Use this test program:
```cpp
void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 3000); // Move 1 tile (24 inches) forward
}
```

#### kP Tuning Loop

| What you see | What it means | What to do |
|---|---|---|
| Robot barely moves, stops way short | kP too low | Increase kP by 1-2 |
| Robot moves but stops 2-3 inches short | kP slightly low | Increase kP by 0.5 |
| Robot reaches target, slight overshoot, settles | ✅ **This is ideal for kP** | Stop, move to kD |
| Robot overshoots and oscillates back and forth | kP too high | Decrease kP by 0.5-1 |
| Robot flies past target violently | kP way too high | Cut kP in half |

> [!IMPORTANT]
> **You WANT slight overshoot when tuning kP alone.** That's the signal that kP is in the right range. kD will clean it up.

#### kD Tuning

Once kP gives slight overshoot, start adding kD:

| What you see | What to do |
|---|---|
| Still overshooting | Increase kD by 0.5 |
| Reaches target cleanly, minimal oscillation | ✅ **Done** |
| Robot slows down too early, crawls to target | kD too high — decrease by 0.25 |
| Robot jitters/vibrates near target | kD way too high — cut in half |

#### Testing distances and directions

After tuning at 24 inches forward, **test at multiple distances and backward**:
```cpp
// Distance tests
chassis.moveToPoint(0, 12, 3000);  // Half tile forward
chassis.moveToPoint(0, 48, 4000);  // 2 tiles forward
chassis.moveToPoint(0, 96, 5000);  // 4 tiles forward

// Backward tests (robot drives in reverse)
chassis.moveToPoint(0, -24, 3000, {.forwards = false});  // 1 tile backward
chassis.moveToPoint(0, -48, 4000, {.forwards = false});  // 2 tiles backward
```

Good PID should work at all distances and in both directions. Forward and backward can behave differently because weight distribution shifts under acceleration/braking. If backward overshoots but forward is fine, kD might need a small bump.

---

### Step 2: Tune Angular (Turning)

**Same process: kP first, then kD, kI = 0.**

Use this test:
```cpp
void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 2000); // Turn 90 degrees clockwise
}
```

#### Angular kP Tuning Loop

| What you see | What it means | What to do |
|---|---|---|
| Robot barely turns, stops 20°+ short | kP too low | Increase by 0.2-0.5 |
| Robot turns but stops 5-10° short | kP slightly low | Increase by 0.1-0.2 |
| Robot reaches 90°, slight overshoot, settles | ✅ **Ideal** | Move to kD |
| Robot passes 90° and oscillates | kP too high | Decrease by 0.1-0.2 |

> [!NOTE]
> Angular kP values are typically **much smaller** than lateral kP. Your current value of 0.863 is in a reasonable range.

#### Angular kD Tuning

Same idea — add kD until the overshoot goes away:

| What you see | What to do |
|---|---|
| Still overshoots past target heading | Increase kD by 0.1 |
| Settles on target cleanly | ✅ **Done** |
| Slows down and crawls the last few degrees | kD too high |
| Vibrates/jitters at the end | kD way too high |

---

## Testing Angles: 90° vs 180° vs 360°

You're right that **larger angles are harder** and reveal problems that small angles hide.

| Test | What it reveals |
|---|---|
| **45°** | Basic sanity check, too easy to catch issues |
| **90°** | Standard test, good starting point |
| **180°** | Reveals overshoot issues — robot builds more speed |
| **270°** | Tests deceleration at high speed |
| **360°** | Hardest — maximum speed, maximum overshoot potential |

### Recommended testing sequence

```cpp
// Test 1: Basic 90° turn
chassis.turnToHeading(90, 2000);

// Test 2: 180° turn (harder)
chassis.turnToHeading(180, 2000);

// Test 3: Negative turns (opposite direction)
chassis.turnToHeading(-90, 2000);  // or 270

// Test 4: Multiple consecutive turns
chassis.turnToHeading(90, 2000);
pros::delay(500);
chassis.turnToHeading(180, 2000);
pros::delay(500);
chassis.turnToHeading(270, 2000);
pros::delay(500);
chassis.turnToHeading(0, 2000);
```

**Tune until 90° is perfect, then verify with 180°.** If 180° overshoots but 90° doesn't, increase kD slightly. The goal is a PID that works for both — it'll be a small compromise.

---

## The Asymmetric Drivetrain Problem

### Why CW and CCW turns aren't equal

This is **extremely common** and is caused by:
1. **Friction differences** — one side of the drivetrain has more friction (tighter gears, bent shaft, motor wear)
2. **Motor differences** — motors aren't identical, one side is slightly stronger
3. **Weight distribution** — robot is heavier on one side
4. **Traction wheel position** — if the traction wheel isn't perfectly centered

### What you can do about it

#### 1. Mechanical fixes (best solution)
- Make sure no shafts are bent
- Check that gears mesh evenly on both sides
- Verify wheels spin freely when motors are detached
- Ensure traction wheel is centered on the drivetrain

#### 2. Match motor behavior
Run this diagnostic during driver control to compare sides:
```cpp
// In opcontrol loop, print to controller:
master.print(0, 0, "L:%d R:%d",
    (int)left_motors.get_actual_velocity()[0],
    (int)right_motors.get_actual_velocity()[0]);
```
If one side is consistently slower at the same power, you know which side has more friction.

#### 3. Tune for the "average"
Since LemLib uses one angular PID for both directions, **tune so both CW and CCW are acceptable** rather than perfect for one direction. If 90° CW is perfect but 90° CCW overshoots by 3°:
- Increase kD slightly — this will slightly undertune CW (maybe -1° error) but fix CCW overshoot
- The result: ±1.5° error in both directions instead of 0° in one and 3° in the other

#### 4. Use the IMU
The IMU already compensates for a lot of drivetrain asymmetry because it measures **actual heading**, not motor encoders. If you're seeing big asymmetry issues, make sure your IMU is:
- Mounted near the center of the robot
- Firmly attached (no vibration)
- Calibrated properly (`chassis.calibrate()`)

---

## Your Current Values — Analysis

```cpp
// Lateral
kP = 4.25    // reasonable
kI = 0       // ✅ correct, leave at 0
kD = 1       // relatively low, might need increasing

// Angular
kP = 0.863   // reasonable
kI = 0       // ✅ correct
kD = 0.235   // seems low — might cause overshoot on large turns
```

### Things to try
- If straight driving overshoots: try lateral kD = **2-3**
- If large turns (180°+) overshoot: try angular kD = **0.4-0.6**

---

## Exit Conditions — Often the Real Problem

Sometimes the robot *looks* badly tuned but the PID is fine — it's just exiting too early or too late.

Your current exit conditions:
```
Lateral:  small error = 1 inch / 100ms,  large error = 3 inches / 500ms
Angular:  small error = 1 degree / 100ms,  large error = 3 degrees / 500ms
```

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot stops short every time | Large error timeout too short | Increase large error timeout |
| Robot sits at target for too long before moving on | Small error timeout too long | Decrease small error timeout |
| Robot oscillates at the end and takes forever to settle | PID is fine but error keeps crossing the threshold | Widen small error range (1 → 2) |

---

## Quick Reference: The Tuning Cheat Sheet

```
1. Set kI = 0, kD = 0
2. Increase kP until you see slight overshoot
3. Increase kD until overshoot goes away
4. Test at multiple distances/angles
5. If there's steady-state error, add tiny kI (try 0.01)
6. Verify in both directions (forward/backward, CW/CCW)
7. Run a full auton to test in real conditions
```

---

## Important Testing Habits

### Always reset pose before each test
```cpp
chassis.setPose(0, 0, 0);
```
Odometry drift from a previous run can make your PID look off when it's actually fine. **Reset every time** so you're always starting from a known state.

### Test back-to-back chained moves
A single move can look perfect in isolation, but **errors compound in chains**. Small overshoot or undershoot on one move shifts the starting position for the next one:
```cpp
void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 3000);    // Move forward 1 tile
    chassis.moveToPoint(24, 24, 3000);   // Move right 1 tile
    chassis.moveToPoint(24, 0, 3000);    // Move back 1 tile
    chassis.moveToPoint(0, 0, 3000);     // Return to start — should end exactly where it started
}
```
If the robot doesn't end up back at the start, your PID has errors that are stacking. This is the real-world test that matters since your autons are all chains of moves.
