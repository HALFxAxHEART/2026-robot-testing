package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics-based simulation, used when {@code Constants.currentMode == Mode.SIM}. Gearing/
 * inertia are rough placeholders (see ShooterIOSim), not measured from the real intake.
 * The spin/roller motor is open-loop percent output on the real robot too, so it's just
 * echoed back directly rather than modeled physically.
 */
public class EvilIntakeIOSim implements EvilIntakeIO {
    private static final double kGearing = 1.0;
    private static final double kMomentOfInertiaKgM2 = 0.005;

    private final DCMotorSim rotationSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getMinion(1), kMomentOfInertiaKgM2, kGearing),
        DCMotor.getMinion(1)
    );

    private final PIDController positionController = new PIDController(3.0, 0, 0);

    private double setpointRotations = 0.0;
    private double spinPercent = 0.0;

    public EvilIntakeIOSim() {
        // Start at rest inside the real robot's soft-limit range instead of at 0 (which is
        // just outside [kReverseSoftLimitRotations, kForwardSoftLimitRotations]).
        rotationSim.setAngle(EvilIntake.kReverseSoftLimitRotations * 2 * Math.PI);
    }

    @Override
    public void updateInputs(EvilIntakeIOInputs inputs) {
        double target = MathUtil.clamp(
            setpointRotations, EvilIntake.kReverseSoftLimitRotations, EvilIntake.kForwardSoftLimitRotations
        );
        double appliedVolts = MathUtil.clamp(
            positionController.calculate(rotationSim.getAngularPositionRotations(), target),
            -12.0, 12.0
        );

        // Emulate the real TalonFX's firmware soft limits so sim behaves like the real robot.
        if (rotationSim.getAngularPositionRotations() >= EvilIntake.kForwardSoftLimitRotations && appliedVolts > 0) {
            appliedVolts = 0;
        } else if (rotationSim.getAngularPositionRotations() <= EvilIntake.kReverseSoftLimitRotations && appliedVolts < 0) {
            appliedVolts = 0;
        }

        rotationSim.setInputVoltage(appliedVolts);
        rotationSim.update(0.02);

        inputs.positionRotations = rotationSim.getAngularPositionRotations();
        inputs.rotorPositionRotations = rotationSim.getAngularPositionRotations();
        inputs.spinAppliedPercent = spinPercent;
    }

    @Override
    public void setRotationPosition(double positionRotations) {
        setpointRotations = positionRotations;
    }

    @Override
    public void setSpinPercent(double percent) {
        spinPercent = percent;
    }
}
