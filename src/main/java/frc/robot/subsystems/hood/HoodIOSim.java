package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics-based simulation, used when {@code Constants.currentMode == Mode.SIM}. Gearing/
 * inertia are rough placeholders (see ShooterIOSim), not measured from the real hood.
 * Gravity torque isn't modeled (generic DCMotorSim, not SingleJointedArmSim) since the
 * hood's actual mechanism geometry isn't known -- close enough to exercise the position
 * control path and see it obey the same soft limits as the real robot.
 */
public class HoodIOSim implements HoodIO {
    private static final double kGearing = 1.0;
    private static final double kMomentOfInertiaKgM2 = 0.001;

    private final DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), kMomentOfInertiaKgM2, kGearing),
        DCMotor.getKrakenX60(1)
    );

    private final PIDController positionController = new PIDController(2.0, 0, 0);

    private boolean closedLoop = false;
    private double closedLoopSetpointRotations = 0.0;
    private double openLoopVolts = 0.0;

    public HoodIOSim() {
        // Start at rest inside the real robot's soft-limit range instead of at 0 (which is
        // outside [kMaxExtensionRotations, kMinExtensionRotations]).
        sim.setAngle(Hood.kMinExtensionRotations * 2 * Math.PI);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        double appliedVolts;
        if (closedLoop) {
            double target = MathUtil.clamp(closedLoopSetpointRotations, Hood.kMaxExtensionRotations, Hood.kMinExtensionRotations);
            appliedVolts = MathUtil.clamp(
                positionController.calculate(sim.getAngularPositionRotations(), target),
                -12.0, 12.0
            );
        } else {
            appliedVolts = openLoopVolts;
        }

        // Emulate the real TalonFX's firmware soft limits so sim behaves like the real robot.
        if (sim.getAngularPositionRotations() >= Hood.kMinExtensionRotations && appliedVolts > 0) {
            appliedVolts = 0;
        } else if (sim.getAngularPositionRotations() <= Hood.kMaxExtensionRotations && appliedVolts < 0) {
            appliedVolts = 0;
        }

        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);

        inputs.positionRotations = sim.getAngularPositionRotations();
        inputs.velocityRotationsPerSec = sim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.appliedVolts = appliedVolts;
        inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setPosition(double positionRotations) {
        closedLoop = true;
        closedLoopSetpointRotations = positionRotations;
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
