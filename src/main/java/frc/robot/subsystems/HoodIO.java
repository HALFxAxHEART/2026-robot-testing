package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for the shooter hood. */
public interface HoodIO {

    @AutoLog
    class HoodIOInputs {
        public double positionRotations = 0.0;
        public double velocityRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(HoodIOInputs inputs) {}

    /** Closed-loop position control, in rotor rotations. */
    default void setPosition(double positionRotations) {}

    /** Open-loop voltage control (used by SysId). */
    default void setVoltage(double volts) {}

    default void stop() {}
}
