# AdvantageKit logging

This codebase logs everything through [AdvantageKit](https://docs.advantagekit.org)
(v26.0.2). Every subsystem that touches real hardware (`Shooter`, `Hood`, `Turret`,
`EvilIntake`, `RollerSystem`) lives in its own folder under `subsystems/` and is split
into:

```
subsystems/shooter/
    Shooter.java          -- the subsystem: setpoints, ready-checks, commands
    ShooterIO.java         -- interface: @AutoLog inputs class + output methods
    ShooterIOTalonFX.java  -- real hardware: the actual TalonFX construction/config
    ShooterIOSim.java      -- physics-based sim, used in desktop simulation
```

(same shape for `hood/`, `turret/`, `intake/`, `rollers/`; `drivetrain/` holds
`CommandSwerveDrivetrain` alone -- see below for why it doesn't get the same split)

- **`XxxIO`** -- an interface listing every input read from hardware (as an `@AutoLog`
  `Inputs` class) and every output command (setVoltage, setVelocity, etc). All methods
  have default no-op bodies, so an empty `new XxxIO() {}` is a valid "do nothing" impl.
- **`XxxIOTalonFX`** (or `IOTalonFXS`) -- the real implementation, talking to the actual
  CAN devices. All the motor construction/config that used to live in the subsystem
  constructor now lives here.
- **`XxxIOSim`** -- a physics-based implementation using WPILib's simulation classes
  (`FlywheelSim`/`DCMotorSim`), so `Mode.SIM` behaves like a real (if roughly-tuned)
  mechanism instead of doing nothing. See "Simulation" below.
- **`Xxx`** (the subsystem itself) -- holds an `XxxIO` and calls `io.updateInputs(inputs)`
  + `Logger.processInputs("Xxx", inputs)` once per cycle in `periodic()`, then reads from
  `inputs` instead of touching hardware directly. All the actual control logic (setpoints,
  ready checks, commands) is unchanged from before -- only *where the hardware calls live*
  moved.

`CommandSwerveDrivetrain` (`subsystems/drivetrain/`) is the one exception -- it's CTRE's
generated swerve code (`TunerSwerveDrivetrain`), which doesn't decompose into this pattern
without a much larger rewrite of CTRE's own control loop. Instead, its `periodic()` just
logs pose/speeds/module states as AdvantageKit *outputs* (`Logger.recordOutput(...)`), so
you still get full AdvantageScope visibility -- you just can't replay the drivetrain's own
control loop the way you can for the other subsystems.

## Simulation (`Mode.SIM`)

Each `XxxIOSim` wraps a WPILib physics sim (`FlywheelSim` for the flywheel/roller-style
mechanisms, `DCMotorSim` for the position-controlled ones) plus a `PIDController` that
stands in for the TalonFX's onboard closed loop (which only exists on real hardware).
**The gearing/moment-of-inertia constants in each `XxxIOSim` are rough placeholders**, not
measured from the real robot -- good enough to see believable motion and exercise every
code path (including the hood's soft limits, which the sim enforces too), not meant to
predict exactly how the real mechanism will respond. Tighten them if you want a closer
match, using the real gear ratios and CAD mass properties if you have them.

`Turret`'s aiming math (`TurretAimCalculator`) doesn't need a sim at all -- it's pure
Java math with no I/O, which is why it already has unit tests instead.

## What this gets you

- **Every match/practice run is recorded** to a `.wpilog` on a USB stick plugged into the
  RIO (`REAL` mode) -- no more losing data because nobody was watching SmartDashboard live.
- **Live viewing**: connect [AdvantageScope](https://docs.advantagescope.org) to the robot
  over NetworkTables and watch any logged field in real time, same as Shuffleboard/Elastic
  but with graphing, 3D field visualization, swerve state display, etc built in.
- **Replay**: pull a `.wpilog` off the robot, point `Constants.currentMode`'s `simMode` at
  `REPLAY`, and re-run the exact match through the current code on your laptop --
  `io.updateInputs()` reads from the log instead of hardware, so all the control logic runs
  identically to how it did on the robot. Useful for "why did the turret aim wrong in match
  14" without needing the robot in front of you.
- **Metadata**: every log records the exact git commit (`BuildConstants.GIT_SHA`) that
  produced it, so you can `git checkout <sha>` to get back to the exact code that ran.

## Modes (`Constants.Mode`)

- `REAL` -- picked automatically when running on the roboRIO. Logs to USB + NT4.
- `SIM` -- the default when not real. Physics-driven via the `XxxIOSim` classes, no log
  file needed to run. This is what `./gradlew simulateJava` uses by default.
- `REPLAY` -- set `Constants.simMode` to this (currently a `private static final` field in
  `Constants.java`, so just edit the constant) when you want to replay a saved log instead
  of running the simulator normally. Requires picking a `.wpilog` file when prompted (or
  passing one via `LogFileUtil`).

## Everything that gets logged

### Automatic (AdvantageKit built-ins -- no code needed)

- **Timestamps** -- `RobotController.getTime()`/`Timer` values, replayed deterministically.
- **`DriverStation` table** -- every value readable via `DriverStation` or the HID classes
  (`CommandXboxController`, etc): alliance, match time, enabled state, all joystick
  axes/buttons/POVs.
- **`RealOutputs`/`ReplayOutputs`** -- WPILib persistent Alerts, console output.
- **`RadioStatus`** -- VH-109 radio connection/bandwidth, every ~5s.
- **`PowerDistribution`** -- per-channel current draw (CTRE PDP or REV PDH on the default
  CAN ID; not currently reconfigured for a non-default ID).
- **`SystemStats`** -- battery voltage, rail status, CAN status, system time, NT client count.
- **`LoggedRobot`/`Logger` performance fields** -- loop timing, GC time, queued-log backlog.

### `Shooter/` (via `ShooterIOInputs`)
- `velocityRPS` -- lead motor velocity
- `appliedVolts`
- `statorCurrentAmps`, `supplyCurrentAmps`
- plus outputs: `Shooter/SysIdState` when running a SysId sweep

### `Hood/` (via `HoodIOInputs`)
- `positionRotations`, `velocityRotationsPerSec`
- `appliedVolts`
- `statorCurrentAmps`
- plus outputs: `Hood/SysIdState` when running a SysId sweep

### `Turret/` (via `TurretIOInputs`)
- `positionMotorRotations`, `velocityMotorRotationsPerSec`
- `appliedVolts`

### `EvilIntake/` (via `EvilIntakeIOInputs`)
- `positionRotations` (mechanism position), `rotorPositionRotations` (raw rotor, what
  `getAngle()` returns -- these differ if there's an internal gear ratio)
- `spinAppliedPercent`
- `statorCurrentAmps` -- feeds stall detection (see `EvilIntake.isStallCondition`); also
  logged as `EvilIntake/Stalled` (a derived output, not a raw input) once the debounced
  check trips

`EvilIntake.periodic()` also owns two behaviors that apply no matter which command is
driving it (button, auto, or the default command below): it forces the rollers to spin
any time the rotation mechanism hasn't reached its commanded target yet (prevents jamming
a piece mid-swing), and it auto-retracts on a stall detected specifically while heading
toward `out` (a wall/robot hit) -- a stall while heading toward `in` (a jammed piece while
retracting) is deliberately left alone for `FunnelAgitate` to react to.

`EvilIntake`'s default command is `FunnelAgitate` (`commands/FunnelAgitate.java`):
whenever nothing else claims the intake and a shot is actively being commanded
(`Shooter.isCommanded()`), it cycles the rack between `in`/`out` to help funnel fuel in,
backing off toward `out` instead of forcing it the moment `EvilIntake.isStalled()` reports
it can't pull in any further.

### `RollerSystem/` (via `RollerSystemIOInputs`)
- `floorVelocityRPS`, `floorAppliedVolts` (the two belt followers aren't logged
  separately since they always mirror the floor roller)

### `Drivetrain/` (manually logged as outputs in `CommandSwerveDrivetrain.periodic()`)
- `Pose`, `Speeds`, `ModuleStates`, `ModuleTargets`

### `Vision/` (manually logged as outputs in `Robot.robotPeriodic()`)
- `Limelight/HasMeasurement`, `Limelight/AppliedPose`, `Limelight/TagCount`
- `LimelightLeft/HasMeasurement`, `LimelightLeft/AppliedPose`, `LimelightLeft/TagCount`

None of the `SmartDashboard.put*` calls still in each subsystem are removed -- they're
kept as-is for whoever's driving to glance at live, in parallel with everything above.

## Viewing logs

Install AdvantageScope (part of the WPILib tools, or standalone from
[advantagescope.org](https://docs.advantagescope.org)). Open a `.wpilog` file directly, or
connect live to the robot's NetworkTables server while it's running. Fields are organized
by subsystem name as listed above, matching the `Logger.processInputs`/`Logger.recordOutput`
calls in each subsystem's `periodic()`.

## Adding a new hardware subsystem

Create a new folder under `subsystems/` and follow the same pattern as `shooter/`/`hood/`/
etc: define `XxxIO` with an `@AutoLog` inputs class + default no-op methods, write
`XxxIOTalonFX` (or whatever vendor hardware) implementing it, optionally write `XxxIOSim`
using `FlywheelSim`/`DCMotorSim`/`SingleJointedArmSim`, and have the subsystem class take
the IO as a constructor argument. In `RobotContainer`, use a
`switch (Constants.currentMode) { case REAL -> ...; case SIM -> ...; case REPLAY -> ...; }`
to pick which one gets built (see the existing subsystem fields for the pattern).
