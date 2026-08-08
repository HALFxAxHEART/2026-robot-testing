# Shoot-on-the-move

Source: FRC 3061 (Huskie Robotics), ["Huskie Physics: Shoot-on-the-move with Equations"](https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805) --
written for the same 2026 "Rebuilt" game this robot plays. Worth reading in full; this doc
only covers what's actually in this repo.

## What's live right now

`TurretAimCalculator` (in `subsystems/turret/`) already does shoot-on-the-move via a
**"virtual target" shift**: it estimates time-of-flight (`distance / estimatedShotSpeedMps`)
and shifts the *aim point* backward by `turretVelocity * timeOfFlight`, then re-aims a
normal static shot at that shifted point. `ShootingMath` then computes flywheel RPS and
hood angle from the distance to that shifted point, exactly like a static shot.

**What changed today**: that lead calculation used to only use the robot's center
velocity (`ChassisSpeeds.vxMetersPerSecond`/`vyMetersPerSecond`), ignoring rotation
entirely. A turret offset from the robot's center of rotation actually picks up extra
tangential velocity whenever the robot spins (`v = v_center + omega x r_offset` -- see the
source post's section 2.3), which matters for a robot that turns while shooting. This is
now included via `ShootOnTheMoveSolver.shooterVelocity()`, wired into
`TurretAimCalculator.solveShooting()`. This didn't need any new calibration -- it's exact
kinematics from a turret offset the code already knew about.

Since this runs inside `Turret.periodic()`, it's exercised automatically in `Mode.SIM`
too -- drive the simulated robot around (translating and rotating) with
`./gradlew simulateJava` and the turret's target motor rotations will visibly react to
both, same as on the real robot.

## What's built but not wired in: `ShootOnTheMoveSolver.solve()`

This implements the source post's core method: rather than shifting an aim point and
re-running a static-shot lookup, it solves directly for the exact launch speed/elevation/
azimuth a moving shooter needs, via 3D vector subtraction (their equations 4-7). It's more
rigorous than the virtual-target trick -- no assumption that "a static shot at the shifted
point" approximates the true correction.

**Why it's not connected to `Shooter`/`Hood` yet**: the algorithm needs a *static shot map*
in physical units -- ball exit speed in m/s and launch angle in degrees, as functions of
distance. What this repo has instead is `ShootingMath.shooterRPSForDistance()` and
`hoodAngleForDistance()`, which map distance directly to motor units (RPS, hood position)
via curves someone already tuned empirically on the real robot.

**Half of the unit conversion is now solved.** `FlywheelPhysics` (in `subsystems/shooter/`)
converts motor RPM to ball exit speed using the real shooter geometry (4in bottom wheel
1:1 off the motor, 2in top wheel geared 2:1 off the bottom wheel -- confirmed to produce
exactly matched surface speeds, which is the actual physical basis for "almost no spin,"
not an approximation). What's still missing:

1. **A real efficiency measurement.** `FlywheelPhysics.kAssumedEfficiency` (0.90) is FRC
   3061's own measured value for their wheels/ball/compression, not this robot's. The
   source post's section 2 describes how to get the real one: slo-mo video (240fps) of the
   ball leaving the shooter, with a reference ruler taped to the frame, at a handful of
   known shots. Compare the video-measured exit speed against
   `FlywheelPhysics.exitSpeedFtPerSec(rpm, 1.0)` (no efficiency applied) to solve for the
   real efficiency fraction.
2. **Hood position -> launch angle.** No relationship between hood mechanism position and
   the ball's actual launch angle is known yet -- same slo-mo video process, measuring
   launch angle instead of speed, at a few different hood positions.

That's real hardware data this repo doesn't have and isn't going to guess at -- getting it
wrong would mean confidently commanding a shot speed/angle based on made-up physics.

### If you do the calibration and want to wire it in

1. Use `FlywheelPhysics.exitSpeedFtPerSec(rpm, realEfficiency)` (with your measured
   efficiency) plus your fitted hood-angle relationship to build `v*(distance)` and
   `elevation*(distance)` -- replacing (or supplementing) `ShootingMath`'s curves.
2. In `Turret.periodic()`, instead of the virtual-target shift, call
   `ShootOnTheMoveSolver.solve(staticShot, turretVelocity, robotPose.getRotation().getRadians())`
   (the `turretVelocity` computation via `ShootOnTheMoveSolver.shooterVelocity()` already
   exists and is reusable as-is).
3. The result's `speedMps()` needs converting back to flywheel RPS (invert whatever
   relationship you found in step 1), and `elevationRad()` back to a hood position.
4. `azimuthRad()` comes back already expressed in the robot/turret frame -- feed it
   straight into the same motor-rotation conversion `solveTurretMotorRotations()` already
   does with the virtual-target's aim angle.

## Real test data now feeds the static shot (partially)

`Turret Raw Values - Sheet1.csv` (repo root) has real distance-vs-RPS test points, hood
held at 0. `ShootingMath.shooterRPSForDistance()` now uses those directly (a perfectly
linear fit, `RPS = 19.6 + 0.0684 x distanceInches`) inside the tested range (42-150in),
falling back to the older quadratic formula outside it.

**Two things to confirm with the team:**
1. **Units**: "Physical Distance" in the sheet is assumed to be inches (cross-checked
   against the old formula, which tracks reasonably closely over this range if so, but
   this is an inference, not a stated fact).
2. **Hood strategy**: this data is hood=0 across the *whole* tested range, but
   `hoodAngleForDistance()` already wants a non-zero hood past 2.2m (~86.6in) -- which
   falls inside the 42-150in tested window. If flat-hood is the actual intended strategy
   across this whole range, `hoodAngleForDistance()`'s 2.2m threshold should move out past
   150in (~3.81m) to match. If hood is supposed to vary in that overlap, this RPS curve
   needs a hood-varying equivalent instead (more test data).

Also worth knowing: the same CSV has a "Simulation stuff"/"Theory" section (X, Theta,
Velocity, RPM F, Pos F...) that looks like a separate physics/trajectory model. Comparing
its `Velocity (ft/s)` column against `FlywheelPhysics` predictions gives a rough efficiency
estimate around 85-88% (vs. the placeholder 90%) -- but since the exact derivation of those
columns isn't confirmed, that's a loose observation, not a wired-in value. That sheet's
"Theory" columns also break down (huge negative values) past physical distance ~294in --
a spreadsheet formula issue, not real data; ignore anything past that point.

## Testing

`ShootOnTheMoveSolverTest` checks the algorithm itself, independent of any calibration:
- Zero shooter velocity is a no-op (returns the static shot unchanged, modulo the
  robot-frame heading conversion).
- **The core physical invariant**: for a handful of shooter velocities, adding the
  shooter's own velocity back onto the solved shot exactly reconstructs the static shot's
  field-frame velocity vector. If the equations were transcribed wrong, this would fail.
- Moving toward the target needs less added shot speed than standing still (sanity check
  on direction, not just magnitude).
- The rotation-velocity helper matches simple hand-computed cross-product cases.

`TurretAimCalculatorTest` has a regression test (`spinningRobotShiftsLeadEvenWithZero
TranslationalVelocity`) confirming the rotation fix is actually wired in -- it would have
failed before today's change, since a spinning-but-not-translating robot used to produce
the exact same lead as a fully stationary one.

`FlywheelPhysicsTest` confirms the "no spin by design" claim mathematically -- the bottom
and top wheel surface speeds are computed independently from the stated diameters/gear
ratio and asserted equal -- plus sanity checks on the exit-speed formula (zero RPM is zero
speed, efficiency scales linearly, ft/s and m/s agree).

`ShootingMathTest` has cases transcribed directly from the CSV's `Cal` column (not derived
from the formula being tested) to confirm the real data was wired in correctly, plus a
case confirming distances just outside the tested range still fall back to the old formula.
