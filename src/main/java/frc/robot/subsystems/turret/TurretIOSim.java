package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics-based simulation, used when {@code Constants.currentMode == Mode.SIM}. Gearing/
 * inertia are rough placeholders (see ShooterIOSim), not measured from the real turret.
 * Approximates the real TalonFXS's Motion Magic with a plain position PID -- good enough
 * to see the turret track a setpoint in sim, not an exact match for the real profile.
 */
public class TurretIOSim implements TurretIO {
    private static final double kGearing = 1.0;
    private static final double kMomentOfInertiaKgM2 = 0.02;

    private final DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getMinion(1), kMomentOfInertiaKgM2, kGearing),
        DCMotor.getMinion(1)
    );

    private final PIDController positionController = new PIDController(8.0, 0, 0);

    private double setpointMotorRotations = 0.0;

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        double appliedVolts = MathUtil.clamp(
            positionController.calculate(sim.getAngularPositionRotations(), setpointMotorRotations),
            -12.0, 12.0
        );
        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);

        inputs.positionMotorRotations = sim.getAngularPositionRotations();
        inputs.velocityMotorRotationsPerSec = sim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.appliedVolts = appliedVolts;
    }

    @Override
    public void setMotionMagicPosition(double motorRotations) {
        setpointMotorRotations = motorRotations;
    }

    @Override
    public void setZeroPosition() {
        sim.setAngle(0);
        setpointMotorRotations = 0;
    }
}
