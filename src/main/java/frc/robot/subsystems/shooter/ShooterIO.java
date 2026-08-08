package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for the shooter flywheel. See TurretAimCalculator-style docs/characterization.md. */
public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ShooterIOInputs inputs) {}

    /** Closed-loop velocity control, in rotations per second. */
    default void setVelocity(double velocityRPS) {}

    /** Open-loop voltage control (used by SysId). */
    default void setVoltage(double volts) {}

    /** Cuts power rather than commanding closed-loop 0. */
    default void stop() {}
}
