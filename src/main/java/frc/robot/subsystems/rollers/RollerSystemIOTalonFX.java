package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Real-hardware implementation: floor roller (leader) + 2 belt motors (followers). */
public class RollerSystemIOTalonFX implements RollerSystemIO {
    private final TalonFX rollerFloor;
    private final TalonFX lowerBelt;
    private final TalonFX upperBelt;

    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut m_voltageControl = new VoltageOut(0);

    public RollerSystemIOTalonFX(CANBus canbus) {
        rollerFloor = new TalonFX(13, canbus);
        lowerBelt = new TalonFX(21, canbus);
        upperBelt = new TalonFX(22, canbus);

        Slot0Configs rollerConfig = new Slot0Configs();
        rollerConfig.kP = 2;

        rollerFloor.getConfigurator().apply(rollerConfig);
        lowerBelt.getConfigurator().apply(rollerConfig);
        upperBelt.getConfigurator().apply(rollerConfig);

        // Set to Brake mode so they stop instantly when power is cut
        rollerFloor.setNeutralMode(NeutralModeValue.Brake);
        lowerBelt.setNeutralMode(NeutralModeValue.Brake);
        upperBelt.setNeutralMode(NeutralModeValue.Brake);

        // Using the correct CTRE Phoenix 6 (v25+) enum for followers
        lowerBelt.setControl(new Follower(13, MotorAlignmentValue.Opposed));
        upperBelt.setControl(new Follower(13, MotorAlignmentValue.Opposed)); // Change to .Aligned if it spins backwards!
    }

    @Override
    public void updateInputs(RollerSystemIOInputs inputs) {
        inputs.floorVelocityRPS = rollerFloor.getVelocity().getValueAsDouble();
        inputs.floorAppliedVolts = rollerFloor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setVelocity(double velocityRPS) {
        rollerFloor.setControl(m_velocityControl.withVelocity(velocityRPS));
    }

    @Override
    public void setVoltage(double volts) {
        rollerFloor.setControl(m_voltageControl.withOutput(volts));
    }
}
