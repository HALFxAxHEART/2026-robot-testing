package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.PhoenixUtil;

/** Real-hardware implementation: 2 opposed TalonFX, lead + follower. */
public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leadShoot;
    private final TalonFX followShoot;

    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut m_voltageControl = new VoltageOut(0);

    public ShooterIOTalonFX(CANBus canbus) {
        // On the Canbus UPPER
        leadShoot = new TalonFX(27, canbus);
        followShoot = new TalonFX(15, canbus);

        // --- LEAD SHOOTER CONFIG ---
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Coast is safer for heavy flywheels

        // PID Configuration
        shooterConfig.Slot0.kP = 0.8;
        shooterConfig.Slot0.kI = 0.0;
        shooterConfig.Slot0.kD = 0.0;
        shooterConfig.Slot0.kV = 0.145;
        shooterConfig.Slot0.kS = 0.0;

        PhoenixUtil.tryUntilOk("Shooter lead (CAN 27)", () -> leadShoot.getConfigurator().apply(shooterConfig));

        TalonFXConfiguration shooterfollowConfig = new TalonFXConfiguration();
        shooterfollowConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        PhoenixUtil.tryUntilOk("Shooter follower (CAN 15)", () -> followShoot.getConfigurator().apply(shooterfollowConfig));
        followShoot.setControl(new Follower(27, MotorAlignmentValue.Opposed));

        // Only broadcast the signals updateInputs() actually reads, at the main loop rate,
        // then drop everything else (on both devices) to a minimal rate -- cuts CAN bus
        // load instead of every signal defaulting to this device's much higher rate.
        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            leadShoot.getVelocity(),
            leadShoot.getMotorVoltage(),
            leadShoot.getStatorCurrent(),
            leadShoot.getSupplyCurrent());
        leadShoot.optimizeBusUtilization();
        followShoot.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.velocityRPS = leadShoot.getVelocity().getValueAsDouble();
        inputs.appliedVolts = leadShoot.getMotorVoltage().getValueAsDouble();
        inputs.statorCurrentAmps = leadShoot.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrentAmps = leadShoot.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setVelocity(double velocityRPS) {
        leadShoot.setControl(m_velocityControl.withVelocity(velocityRPS));
    }

    @Override
    public void setVoltage(double volts) {
        leadShoot.setControl(m_voltageControl.withOutput(volts));
    }

    @Override
    public void stop() {
        leadShoot.stopMotor(); // Safely cuts power to the motor rather than commanding 0 RPS closed-loop
    }
}
