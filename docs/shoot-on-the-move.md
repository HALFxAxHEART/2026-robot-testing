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
via curves someone already tuned empirically on the real robot. There's no known conversion
from those motor units back to physical ball speed/angle -- that requires measuring your
actual flywheel radius and the real relationship between hood position and shot angle.

The source post's section 2 describes exactly how to get that: slo-mo video (240fps) of
the ball leaving the shooter, with a reference ruler taped to the shooter frame, at a
handful of known shots (their example: 4 shots, high/low x fast/slow). That's real
hardware data this repo doesn't have and I'm not going to guess at -- getting it wrong
would mean confidently commanding a shot speed/angle based on made-up physics.

### If you do the calibration and want to wire it in

1. Replace (or supplement) `ShootingMath`'s distance-based curves with `v*(distance)` in
   m/s and `elevation*(distance)` in radians, fit from your slo-mo measurements.
2. In `Turret.periodic()`, instead of the virtual-target shift, call
   `ShootOnTheMoveSolver.solve(staticShot, turretVelocity, robotPose.getRotation().getRadians())`
   (the `turretVelocity` computation via `ShootOnTheMoveSolver.shooterVelocity()` already
   exists and is reusable as-is).
3. The result's `speedMps()` needs converting back to flywheel RPS (invert whatever
   relationship you found in step 1), and `elevationRad()` back to a hood position.
4. `azimuthRad()` comes back already expressed in the robot/turret frame -- feed it
   straight into the same motor-rotation conversion `solveTurretMotorRotations()` already
   does with the virtual-target's aim angle.

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
