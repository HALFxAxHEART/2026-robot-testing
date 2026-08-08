package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Hood extends SubsystemBase {
    // Same numbers RobotContainer clamps the commanded angle to -- kept here as the
    // single source of truth so the firmware soft limits and the app-level clamp agree.
    public static final double kMinExtensionRotations = -0.12890625; // resting/minimum position
    public static final double kMaxExtensionRotations = -2.7734375;  // absolute max the hood can physically go

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private double setpoint = 0.0;

    /**
     * NOT bound to any button by default -- run this deliberately, with someone watching
     * the hood, not mid-match. See docs/characterization.md. Kept slow/short since the
     * hood only has ~2.6 rotations of travel between its soft limits (which will now
     * safely stop a sweep instead of letting it run into the hard stop).
     */
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Second), // slow ramp -- short travel range
            Volts.of(2),               // small step -- short travel range
            Second.of(5),              // short timeout so it can't run away
            state -> Logger.recordOutput("Hood/SysIdState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            voltage -> runVolts(voltage.in(Volts)),
            null, // no log consumer -- AdvantageKit already records everything via updateInputs
            this
        )
    );

    public Hood(HoodIO io){
        this.io = io;
    }

    public boolean atSetpoint(){
        return Math.abs(getAngle() - setpoint) <= 0.5;
    }

    /** @return hood position */
    public double getAngle(){
       return inputs.positionRotations;
    }

    // hood go go!
    public void goTo(double position){
        setpoint = position;
        io.setPosition(setpoint);
    }

    public void incrementPositionBy(double revolutions) {
        setpoint += revolutions;
        io.setPosition(setpoint);
    }

    public void stop(){
        io.stop();
    }

    /** Open-loop voltage control, used by the SysId routine. */
    public void runVolts(double volts) {
        io.setVoltage(volts);
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
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        SmartDashboard.putNumber("Hood Angle", inputs.positionRotations);
        SmartDashboard.putNumber("Target Angle", setpoint);
    }
}
