# Hardware characterization runbook

This covers the two things in the codebase that were tuned by hand/guess and need
real numbers from the actual robot before you trust them: the Shooter/Hood feedforward
gains, and the EvilIntake soft limits. Nothing in this doc can be done without hands on
the robot -- that's the point of writing it down instead of guessing.

## 1. SysId characterization (Shooter, Hood)

`Shooter` and `Hood` each have a `SysIdRoutine` wired up (same pattern the drivetrain
already uses in `CommandSwerveDrivetrain`), exposing `sysIdQuasistatic(direction)` and
`sysIdDynamic(direction)` Commands. They are **not bound to any button** on purpose --
running a SysId sweep mid-match would be bad. Wire them up temporarily.

### Before you run anything
- Robot on blocks (drive) is irrelevant here, but for the **Shooter**: keep hands, hair,
  and loose clothing away from the flywheel, and make sure whatever the shooter feeds
  into is clear -- it will spin up for real.
- For the **Hood**: it now has firmware soft limits (`Hood.kMinExtensionRotations` /
  `kMaxExtensionRotations`) so a sweep should stop itself at the physical range instead
  of grinding into the hard stop. Still, have a hand on the robot disable / E-stop the
  first time you run it, in case those limits are wrong.
- Have a teammate at the driver station the whole time.

### Running a sweep
1. Temporarily bind the four SysId commands to spare buttons in `RobotContainer`
   (test-mode only is safest), e.g.:
   ```java
   RobotModeTriggers.test().and(joystick.a()).whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
   RobotModeTriggers.test().and(joystick.b()).whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
   RobotModeTriggers.test().and(joystick.x()).whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
   RobotModeTriggers.test().and(joystick.y()).whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
   ```
   (Swap `shooter` for `hood` to characterize the hood instead -- do them one at a time.)
2. Enable **Test mode** in the Driver Station, run each of the four directions once
   (quasistatic forward, quasistatic reverse, dynamic forward, dynamic reverse), letting
   each finish or timeout on its own. Disable between each one.
3. The routines log via CTRE's `SignalLogger` (the same one the drivetrain SysId already
   uses), so pull the log the same way you already do for drivetrain characterization:
   Phoenix Tuner X → Log Extractor, convert the `.hoot` to `.wpilog`, then open it in the
   WPILib **SysId** tool (in the WPILib tools suite) and pick the mechanism type
   (Simple/Elevator/Arm -- Shooter is a flywheel so use Simple; Hood may need Arm if
   gravity torque matters for it).
4. SysId will output `kS`, `kV`, `kA` (and `kG` if you used Arm/Elevator). Plug those into:
   - `Shooter.java` -- the `Slot0Configs`/`shooterConfig.Slot0` block in the constructor.
   - `Hood.java` -- the `hoodPID` block in the constructor.
5. Remove the temporary button bindings before you ship the change.

## 2. EvilIntake soft limits

`EvilIntake.java` currently sets:
```java
intakeConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = EvilIntakePosition.out.getAngle() + 0.5;
intakeConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = EvilIntakePosition.in.getAngle() - 0.1;
```
Those margins (`+0.5`, `-0.1` rotor rotations) are guesses layered on top of the two
setpoints the code already commands (`in = 0.36`, `out = 17`) -- **not measured against
the real hard stops.** There's also an unresolved question in the same file about
whether the motor direction is inverted (see the commented-out `InvertedValue` lines),
which determines whether "Forward" and "Reverse" in the config even correspond to "out"
and "in" the way you'd expect.

To get real numbers:
1. In Phoenix Tuner X, open the intake rotation motor (CAN ID 11) and use the manual
   control tab to jog it **slowly, in small steps**, watching the reported position.
2. Confirm which direction is "out" vs "in" first -- resolve the inversion question
   before anything else, since it flips which config field is which limit.
3. Jog toward the deployed ("out") hard stop until it just stops moving/stalls, note
   the position, then back off. Set `ForwardSoftLimitThreshold` (or `Reverse`, depending
   on what step 2 found) to that position minus a safety margin (a few percent of the
   full range, not fixed to `+0.5`).
4. Repeat for the stowed ("in") hard stop.
5. Update the two threshold lines in `EvilIntake.java` with the real values and delete
   the "these margins are a guess" comment once they're verified.
6. While you're in there: the stator current limit comment says "5 Amps" but the code
   sets 40A -- decide what you actually want it to do when the intake hits a piece or a
   finger, and make the comment and the number agree.
