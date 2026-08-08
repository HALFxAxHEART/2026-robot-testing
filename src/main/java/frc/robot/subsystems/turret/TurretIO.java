package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for the turret rotation motor. */
public interface TurretIO {

    @AutoLog
    class TurretIOInputs {
        public double positionMotorRotations = 0.0;
        public double velocityMotorRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(TurretIOInputs inputs) {}

    /** Motion Magic position control, in motor rotations. */
    default void setMotionMagicPosition(double motorRotations) {}

    /** Zeroes the motor's internal position reference. */
    default void setZeroPosition() {}
}
