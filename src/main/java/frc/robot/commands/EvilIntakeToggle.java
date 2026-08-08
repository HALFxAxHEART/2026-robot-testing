package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EvilIntakePosition;
import frc.robot.subsystems.intake.EvilIntake;

/**
 * Toggles the intake between deployed and stowed each time this command is scheduled.
 * NOTE: because isFinished() never returns true, binding this with .onTrue() only
 * fires initialize() on the very first press -- the CommandScheduler treats later
 * presses as a no-op since the command is already running. Bind with
 * .toggleOnTrue(new EvilIntakeToggle(evilIntake)) instead so each press actually
 * schedules/cancels it.
 */
public class EvilIntakeToggle extends Command{
    /** Give the intake this long to rotate out before we spin the rollers. */
    private static final double kSpinDelaySeconds = 0.125;

    EvilIntake evilIntake;
    EvilIntakePosition evilTarget;
    boolean OutNow = true;
    private final Timer timer = new Timer();

        public EvilIntakeToggle(EvilIntake evilIntake){
            this.evilIntake = evilIntake;

            addRequirements(evilIntake);
        }

        @Override
        public void initialize() {
            if (OutNow == false){
                evilIntake.evilyummy(EvilIntakePosition.out);
                timer.restart();
                OutNow = true;
            }
            else {
                evilIntake.evileryummy(0);
                evilIntake.evilyummy(EvilIntakePosition.in);
                OutNow = false;
            }
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            if (OutNow && timer.hasElapsed(kSpinDelaySeconds)) {
                evilIntake.evileryummy(-1);
            }
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted){}

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false; // Has no end condition
        }
}