package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Real-hardware implementation: single TalonFX. */
public class HoodIOTalonFX implements HoodIO {
    private final TalonFX hood;

    private final PositionVoltage m_positionControl = new PositionVoltage(0).withSlot(0);
    private final VoltageOut m_voltageControl = new VoltageOut(0);

    public HoodIOTalonFX(CANBus canbus) {
        hood = new TalonFX(20, canbus);

        // --- HOOD CONFIG ---
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Enforce the same range at the motor controller so a bad setpoint from
        // anywhere else in the code can't drive the hood into its hard stop.
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Hood.kMinExtensionRotations;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Hood.kMaxExtensionRotations;

        hood.getConfigurator().apply(hoodConfig);

        Slot0Configs hoodPID = new Slot0Configs();
        hoodPID.kP = 2;
        hood.getConfigurator().apply(hoodPID);

        // Only broadcast the signals updateInputs() actually reads, at the main loop rate,
        // then drop everything else to a minimal rate -- cuts CAN bus load instead of every
        // signal defaulting to this device's much higher rate.
        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            hood.getPosition(),
            hood.getVelocity(),
            hood.getMotorVoltage(),
            hood.getStatorCurrent());
        hood.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionRotations = hood.getPosition().getValueAsDouble();
        inputs.velocityRotationsPerSec = hood.getVelocity().getValueAsDouble();
        inputs.appliedVolts = hood.getMotorVoltage().getValueAsDouble();
        inputs.statorCurrentAmps = hood.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setPosition(double positionRotations) {
        hood.setControl(m_positionControl.withPosition(positionRotations));
    }

    @Override
    public void setVoltage(double volts) {
        hood.setControl(m_voltageControl.withOutput(volts));
    }

    @Override
    public void stop() {
        hood.set(0);
    }
}
