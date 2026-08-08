package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for the floor roller + belt feed motors. */
public interface RollerSystemIO {

    @AutoLog
    class RollerSystemIOInputs {
        public double floorVelocityRPS = 0.0;
        public double floorAppliedVolts = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(RollerSystemIOInputs inputs) {}

    /** Closed-loop velocity control for the floor roller (belts follow it). */
    default void setVelocity(double velocityRPS) {}

    /** Open-loop voltage control for the floor roller. */
    default void setVoltage(double volts) {}
}
