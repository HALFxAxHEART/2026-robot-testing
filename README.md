# 2026 Robot

WPILib Java robot code for the 2026 season ("Rebuilt"). Swerve drive (CTRE Phoenix 6,
generated via Tuner X) with a turret-mounted flywheel shooter, hood, ball intake, and
feed rollers.

## Building and testing

```
./gradlew build       # compiles + runs the JUnit test suite
./gradlew test         # just the tests
./gradlew simulateJava # desktop simulation (physics-based, see below)
```

Deploying to the robot is the usual WPILib flow (`WPILib: Deploy Robot Code` in VS Code,
or `./gradlew deploy`).

## Code layout

Each hardware subsystem lives in its own folder under `src/main/java/frc/robot/subsystems/`,
split into an IO interface + real implementation + simulation implementation, all logged
through [AdvantageKit](https://docs.advantagekit.org):

```
subsystems/shooter/
    Shooter.java          -- setpoints, ready-checks, commands
    ShooterIO.java          -- interface: @AutoLog inputs + output methods
    ShooterIOTalonFX.java   -- real hardware
    ShooterIOSim.java       -- physics-based simulation
```

Same shape for `hood/`, `turret/`, `intake/`, `rollers/`. `drivetrain/` holds
`CommandSwerveDrivetrain` (CTRE-generated swerve code) on its own -- it doesn't split into
the IO pattern, see `docs/advantagekit.md` for why.

`subsystems/turret/` also has `TurretAimCalculator` (the pure aim-solving math) and
`ShootOnTheMoveSolver` (shoot-on-the-move physics) -- both plain Java with no hardware
dependency, which is why they have unit tests instead of needing simulation.

## Docs

- **[docs/advantagekit.md](docs/advantagekit.md)** -- the IO/logging pattern, the
  REAL/SIM/REPLAY mode system, and a full inventory of everything that gets logged.
- **[docs/characterization.md](docs/characterization.md)** -- how to run SysId on the
  shooter/hood and find the real intake soft-limit positions (needs hands on the robot).
- **[docs/shoot-on-the-move.md](docs/shoot-on-the-move.md)** -- the shoot-on-the-move
  math, what's live vs. what's waiting on shooter calibration.

## Testing without hardware

`src/test/java` has JUnit tests for everything that's pure math (turret aiming, shooter/
hood formulas, shoot-on-the-move). Run `./gradlew test` before pushing. `./gradlew
simulateJava` runs the full robot code against physics-based simulation (`Mode.SIM`) --
useful for exercising control logic (including shoot-on-the-move while the simulated
robot moves) without a real robot.
