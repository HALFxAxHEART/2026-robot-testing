package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for the intake's rotation motor + spin roller motor. */
public interface EvilIntakeIO {

    @AutoLog
    class EvilIntakeIOInputs {
        public double positionRotations = 0.0;
        public double rotorPositionRotations = 0.0;
        public double spinAppliedPercent = 0.0;
        /** Rotation motor's stator current -- used for stall detection, see EvilIntake. */
        public double statorCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(EvilIntakeIOInputs inputs) {}

    /** Closed-loop position control for the rotation motor, in rotor rotations. */
    default void setRotationPosition(double positionRotations) {}

    /** Open-loop percent output for the spin/roller motor. */
    default void setSpinPercent(double percent) {}
}
