package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Physics-based simulation, used when {@code Constants.currentMode == Mode.SIM}. The
 * moment of inertia/gearing below are rough placeholders, not measured from the real
 * robot -- good enough to exercise code paths and see believable spin-up behavior in
 * AdvantageScope, not meant to predict the real shooter's exact response.
 */
public class ShooterIOSim implements ShooterIO {
    private static final double kGearing = 1.0;
    private static final double kMomentOfInertiaKgM2 = 0.01;

    private final FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), kMomentOfInertiaKgM2, kGearing),
        DCMotor.getKrakenX60(1)
    );

    // Simple velocity loop closing over the physics sim, standing in for the TalonFX's
    // onboard closed loop (which only exists on real hardware).
    private final PIDController velocityController = new PIDController(0.4, 0, 0);

    private boolean closedLoop = false;
    private double closedLoopSetpointRPS = 0.0;
    private double openLoopVolts = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
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

        inputs.velocityRPS = sim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.appliedVolts = appliedVolts;
        inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
        inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
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

    @Override
    public void stop() {
        closedLoop = false;
        openLoopVolts = 0.0;
    }
}
