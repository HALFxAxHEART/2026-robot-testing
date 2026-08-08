package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
        shooterConfig.Slot0.kP = 0.4;
        shooterConfig.Slot0.kI = 0.0;
        shooterConfig.Slot0.kD = 0.0005;
        shooterConfig.Slot0.kV = 0.16;
        shooterConfig.Slot0.kS = 0.0;

        leadShoot.getConfigurator().apply(shooterConfig);

        // --- SHOOTER CONFIG ---
        Slot0Configs shooterPID = new Slot0Configs();
        shooterPID.kP = .8; // Changed to 2.0 for smoother recovery
        shooterPID.kI = 0.0;
        shooterPID.kD = 0; //.0005
        shooterPID.kV = 0.145; // .16
        shooterPID.kS = 0;
        //shooterPID.kA = 1.0;

        leadShoot.getConfigurator().apply(shooterPID);
        //followShoot.getConfigurator().apply(shooterPID);

        TalonFXConfiguration shooterfollowConfig = new TalonFXConfiguration();
        shooterfollowConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followShoot.getConfigurator().apply(shooterfollowConfig);
        followShoot.setControl(new Follower(27, MotorAlignmentValue.Opposed));
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
