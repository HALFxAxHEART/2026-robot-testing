package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    /** We store the target RPS here so the plus/minus buttons work correctly */
    private double setpoint = 0.0;

    /**
     * NOT bound to any button by default -- run this deliberately, with someone watching
     * the flywheel and a hand on the E-stop, not mid-match. See docs/characterization.md.
     * Ramp/step voltage are kept conservative since this is a heavy flywheel, not a drivetrain.
     */
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // default ramp rate (1 V/s)
            Volts.of(4), // reduced dynamic step to avoid slamming the flywheel to full speed
            null,        // default timeout (10 s)
            state -> Logger.recordOutput("Shooter/SysIdState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            voltage -> runVolts(voltage.in(Volts)),
            null, // no log consumer -- AdvantageKit already records everything via updateInputs
            this
        )
    );

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    /** Checks if the shooter is up to speed and ready to fire */
    public boolean isShooterReady(double range) {
        // If we aren't trying to shoot, the shooter isn't "ready"
        if (setpoint == 0.0) {
            return false;
        }
        return Math.abs(inputs.velocityRPS - setpoint) <= range;
    }

    /** Updates the targetRPS and sends it to the shooter */
    public void goShoot(double RPS) {
        this.setpoint = RPS;
        io.setVelocity(setpoint);
    }

    /** Adds the given RPS to the shooter's current setpoint. */
    public void incrementVelocityBy(double RPS) {
        this.setpoint += RPS;
        io.setVelocity(setpoint);
    }

    public void stop() {
        setpoint = 0.0; // Reset target speed
        io.stop();
    }

    /** Open-loop voltage control, used by the SysId routine. */
    public void runVolts(double volts) {
        io.setVoltage(volts);
    }

    /** * Returns a command that drives the shooter, then stops it at command end.
     * SOTM MAGIC: Because RPS is a DoubleSupplier, it will constantly recalculate
     * the optimal speed based on robot velocity while the button is held down!
     */
    public Command shoot(DoubleSupplier RPS) {
        return this.runEnd(
            () -> goShoot(RPS.getAsDouble()),
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
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        SmartDashboard.putNumber("Shooter/Velocity_RPS", inputs.velocityRPS);
        SmartDashboard.putNumber("Shooter/Target_Velocity_RPS", setpoint);
        SmartDashboard.putBoolean("Shooter/Ready", isShooterReady(2));
    }
}
