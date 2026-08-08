package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Hood extends SubsystemBase {
    // Same numbers RobotContainer clamps the commanded angle to -- kept here as the
    // single source of truth so the firmware soft limits and the app-level clamp agree.
    public static final double kMinExtensionRotations = -0.12890625; // resting/minimum position
    public static final double kMaxExtensionRotations = -2.7734375;  // absolute max the hood can physically go

    TalonFX hood;

    private double setpoint = 0.0;

    private final VoltageOut m_sysIdControl = new VoltageOut(0);

    /**
     * NOT bound to any button by default -- run this deliberately, with someone watching
     * the hood, not mid-match. See docs/characterization.md. Kept slow/short since the
     * hood only has ~2.6 rotations of travel between its soft limits (which will now
     * safely stop a sweep instead of letting it run into the hard stop).
     */
    private final SysIdRoutine m_sysIdRoutine;

    public Hood(CANBus canbus){
        hood = new TalonFX(20, canbus);

        m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second), // slow ramp -- short travel range
                Volts.of(2),               // small step -- short travel range
                Second.of(5),              // short timeout so it can't run away
                state -> SignalLogger.writeString("SysIdHood_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                voltage -> hood.setControl(m_sysIdControl.withOutput(voltage.in(Volts))),
                null,
                this
            )
        );

        // --- HOOD CONFIG ---
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Enforce the same range at the motor controller so a bad setpoint from
        // anywhere else in the code can't drive the hood into its hard stop.
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMinExtensionRotations;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kMaxExtensionRotations;

        hood.getConfigurator().apply(hoodConfig);

        Slot0Configs hoodPID = new Slot0Configs();
        hoodPID.kP = 2;
        hood.getConfigurator().apply(hoodPID);
    }

    public boolean atSetpoint(){
        return Math.abs(getAngle() - setpoint) <= 0.5;
    }

    /** @return hood position */
    public double getAngle(){
       return hood.getPosition().getValueAsDouble();
    }

    // hood go go!
    public void goTo(double position){
        setpoint = position;
        PositionVoltage hoodRequest = new PositionVoltage(setpoint).withSlot(0);
        hood.setControl(hoodRequest);
    }
    
    public void incrementPositionBy(double revolutions) {
        setpoint += revolutions;
        PositionVoltage hoodRequest = new PositionVoltage(setpoint).withSlot(0);
        hood.setControl(hoodRequest);
    }

    public void stop(){
        hood.set(0);
    }

    public Command hoodgo(DoubleSupplier posH) {
        return this.runEnd(
            () -> goTo(posH.getAsDouble()),
            () -> stop()
        );
    }

    /** Runs a SysId quasistatic (slow ramp) sweep. See docs/characterization.md before using. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /** Runs a SysId dynamic (step voltage) sweep. See docs/characterization.md before using. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Hood Angle", hood.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Target Angle", setpoint);
    }


    
}
