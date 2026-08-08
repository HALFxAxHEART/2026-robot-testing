# AdvantageKit logging

This codebase logs everything through [AdvantageKit](https://docs.advantagekit.org)
(v26.0.2). Every subsystem that touches real hardware (`Shooter`, `Hood`, `Turret`,
`EvilIntake`, `RollerSystem`) is split into:

- **`XxxIO`** -- an interface listing every input read from hardware (as an `@AutoLog`
  `Inputs` class) and every output command (setVoltage, setVelocity, etc).
- **`XxxIOTalonFX`** (or `IOTalonFXS`) -- the real implementation, talking to the actual
  CAN devices. All the motor construction/config that used to live in the subsystem
  constructor now lives here.
- **`Xxx`** (the subsystem itself) -- holds an `XxxIO` and calls `io.updateInputs(inputs)`
  + `Logger.processInputs("Xxx", inputs)` once per cycle in `periodic()`, then reads from
  `inputs` instead of touching hardware directly. All the actual control logic (setpoints,
  ready checks, commands) is unchanged from before -- only *where the hardware calls live*
  moved.

`CommandSwerveDrivetrain` is the one exception -- it's CTRE's generated swerve code
(`TunerSwerveDrivetrain`), which doesn't decompose into this pattern without a much larger
rewrite of CTRE's own control loop. Instead, its `periodic()` just logs pose/speeds/module
states as AdvantageKit *outputs* (`Logger.recordOutput(...)`), so you still get full
AdvantageScope visibility -- you just can't replay the drivetrain's own control loop the
way you can for the other subsystems.

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
- `SIM` -- the default when not real. Normal desktop simulation, physics-driven, no log
  file needed to run. This is what `./gradlew simulateJava` uses by default.
- `REPLAY` -- set `Constants.simMode` to this (currently a `private static final` field in
  `Constants.java`, so just edit the constant) when you want to replay a saved log instead
  of running the simulator normally. Requires picking a `.wpilog` file when prompted (or
  passing one via `LogFileUtil`).

## Viewing logs

Install AdvantageScope (part of the WPILib tools, or standalone from
[advantagescope.org](https://docs.advantagescope.org)). Open a `.wpilog` file directly, or
connect live to the robot's NetworkTables server while it's running. Fields are organized
by subsystem name (`Shooter/...`, `Hood/...`, `Turret/...`, `EvilIntake/...`,
`RollerSystem/...`, `Drivetrain/...`, `Vision/...`), matching the `Logger.processInputs`/
`Logger.recordOutput` calls in each subsystem's `periodic()`.

## Adding a new hardware subsystem

Follow the same three-file pattern as `Shooter`/`Hood`/etc: define `XxxIO` with an
`@AutoLog` inputs class + default no-op methods, write `XxxIOTalonFX` (or whatever vendor
hardware) implementing it, and have the subsystem class take the IO as a constructor
argument. In `RobotContainer`, construct the real IO when `Constants.currentMode == Mode.REAL`
and an anonymous dummy (`new XxxIO() {}`) otherwise.
