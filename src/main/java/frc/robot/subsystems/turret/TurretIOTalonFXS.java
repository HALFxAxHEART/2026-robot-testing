package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.PhoenixUtil;

/** Real-hardware implementation: single TalonFXS (Minion) driving the turret ring gear. */
public class TurretIOTalonFXS implements TurretIO {
    private final TalonFXS m_turretMotor;
    private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);

    public TurretIOTalonFXS(CANBus canbus) {
        m_turretMotor = new TalonFXS(16, canbus);

        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        config.Slot0.kP = 8;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0;

        config.MotionMagic.MotionMagicCruiseVelocity = 600.0;
        config.MotionMagic.MotionMagicAcceleration = 60.0;
        config.MotionMagic.MotionMagicJerk = 1600.0;

        PhoenixUtil.tryUntilOk("Turret (CAN 16)", () -> m_turretMotor.getConfigurator().apply(config));
        m_turretMotor.setPosition(0);

        // Only broadcast the signals updateInputs() actually reads, at the main loop rate,
        // then drop everything else to a minimal rate -- cuts CAN bus load instead of every
        // signal defaulting to this device's much higher out-of-the-box rate.
        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            m_turretMotor.getPosition(),
            m_turretMotor.getVelocity(),
            m_turretMotor.getMotorVoltage());
        m_turretMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // No .refresh() here -- Phoenix6 keeps these signals fresh in the background at the
        // rate configured above, so reading the cached value avoids blocking the loop on a
        // synchronous CAN round-trip every single cycle.
        inputs.positionMotorRotations = m_turretMotor.getPosition().getValueAsDouble();
        inputs.velocityMotorRotationsPerSec = m_turretMotor.getVelocity().getValueAsDouble();
        inputs.appliedVolts = m_turretMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setMotionMagicPosition(double motorRotations) {
        m_turretMotor.setControl(m_motionMagic.withPosition(motorRotations));
    }

    @Override
    public void setZeroPosition() {
        m_turretMotor.setPosition(0.0);
    }
}
