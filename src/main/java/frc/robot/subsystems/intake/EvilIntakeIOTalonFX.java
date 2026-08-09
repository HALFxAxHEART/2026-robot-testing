package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.PhoenixUtil;

/** Real-hardware implementation: rotation TalonFX + spin TalonFX. */
public class EvilIntakeIOTalonFX implements EvilIntakeIO {
    private final TalonFX intakeMotor;
    private final TalonFX spinMotor;

    // Define the control request once up here to save Garbage Collection overhead!
    private final PositionVoltage rotationRequest = new PositionVoltage(0).withSlot(0);

    public EvilIntakeIOTalonFX(int intakeID, int spinID, CANBus canbus) {
        intakeMotor = new TalonFX(intakeID, canbus);
        spinMotor = new TalonFX(spinID, canbus);

        // --- INTAKE MOTOR CONFIGURATION ---
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        // 1. Set to Coast Mode
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // 2. Hard ceiling on motor current -- purely electrical protection, caps torque
        // output so the motor/breaker survive a stall, doesn't change what the mechanism
        // does. NOTE: comment used to say "5 Amps" but this was set to 40 -- flagging in
        // case 40A wasn't actually intended; confirm against what you want it to do when
        // it hits a piece/finger.
        //
        // Separately, EvilIntake.kStallCurrentAmps (25A, well under this 40A ceiling) is
        // the BEHAVIORAL stall-detection threshold that triggers auto-retract -- see
        // EvilIntake.isStallCondition(). This limit here still needs to sit above that,
        // since the motor should be free to actually draw stall-detection-level current
        // rather than get capped before ever reaching it.
        intakeConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // 3. Soft limits around the only two commanded setpoints (in = 0.36, out = 17 rotor rotations).
        // NOTE: these margins are a guess -- verify against the real hard stops before trusting them,
        // and double check Forward vs Reverse isn't backwards once you confirm motor inversion below.
        intakeConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        intakeConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = EvilIntake.kForwardSoftLimitRotations;
        intakeConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        intakeConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = EvilIntake.kReverseSoftLimitRotations;

        /* intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; */
        // yeah idk will do research on if its inverted or not lol

        // 4. PID Tuning (Uncommented and fixed variable name to intakeConfig)
        intakeConfig.Slot0.kP = 3;
       // intakeConfig.Slot0.kD = 0;
       // intakeConfig.Slot0.kV = 1;
       // intakeConfig.Slot0.kG = 1.8;
        intakeConfig.Slot0.kS = 15;
        // pdvga......................................................!!

        // Apply configs to the intake motor
        PhoenixUtil.tryUntilOk("EvilIntake rotation (CAN " + intakeID + ")", () -> intakeMotor.getConfigurator().apply(intakeConfig));

        // --- SPIN MOTOR CONFIGURATION ---
        TalonFXConfiguration spinConfig = new TalonFXConfiguration();
        PhoenixUtil.tryUntilOk("EvilIntake spin (CAN " + spinID + ")", () -> spinMotor.getConfigurator().apply(spinConfig));

        // Only broadcast the signals updateInputs() actually reads, at the main loop rate,
        // then drop everything else (on both devices) to a minimal rate -- cuts CAN bus
        // load instead of every signal defaulting to this device's much higher rate.
        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            intakeMotor.getPosition(),
            intakeMotor.getRotorPosition(),
            intakeMotor.getStatorCurrent());
        intakeMotor.optimizeBusUtilization();
        spinMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(EvilIntakeIOInputs inputs) {
        inputs.positionRotations = intakeMotor.getPosition().getValueAsDouble();
        inputs.rotorPositionRotations = intakeMotor.getRotorPosition().getValueAsDouble();
        inputs.spinAppliedPercent = spinMotor.get();
        inputs.statorCurrentAmps = intakeMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setRotationPosition(double positionRotations) {
        intakeMotor.setControl(rotationRequest.withPosition(positionRotations));
    }

    @Override
    public void setSpinPercent(double percent) {
        spinMotor.set(percent);
    }
}
