package frc.robot.subsystems.rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Physics-based simulation, used when {@code Constants.currentMode == Mode.SIM}. Gearing/
 * inertia are rough placeholders (see ShooterIOSim), not measured from the real rollers.
 * Only the floor roller is modeled -- the belts are followers on the real robot too, so
 * they don't need their own physics.
 */
public class RollerSystemIOSim implements RollerSystemIO {
    private static final double kGearing = 1.0;
    private static final double kMomentOfInertiaKgM2 = 0.001;

    private final FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), kMomentOfInertiaKgM2, kGearing),
        DCMotor.getKrakenX60(1)
    );

    private final PIDController velocityController = new PIDController(2.0, 0, 0);

    private boolean closedLoop = false;
    private double closedLoopSetpointRPS = 0.0;
    private double openLoopVolts = 0.0;

    @Override
    public void updateInputs(RollerSystemIOInputs inputs) {
        double appliedVolts;
        if (closedLoop) {
            appliedVolts = MathUtil.clamp(
                velocityController.calculate(sim.getAngularVelocityRadPerSec() / (2 * Math.PI), closedLoopSetpointRPS),
                -12.0, 12.0
            );
        } else {
            appliedVolts = openLoopVolts;
        }
        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);

        inputs.floorVelocityRPS = sim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.floorAppliedVolts = appliedVolts;
    }

    @Override
    public void setVelocity(double velocityRPS) {
        closedLoop = true;
        closedLoopSetpointRPS = velocityRPS;
    }

    @Override
    public void setVoltage(double volts) {
        closedLoop = false;
        openLoopVolts = volts;
    }
}
