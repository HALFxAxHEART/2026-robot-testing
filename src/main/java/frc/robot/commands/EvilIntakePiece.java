package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EvilIntakePosition;
import frc.robot.subsystems.EvilIntake;

public class EvilIntakePiece extends Command{
    /** Give the intake this long to rotate out before we spin the rollers. */
    private static final double kSpinDelaySeconds = 0.125;

    EvilIntake evilIntake;
    EvilIntakePosition evilTarget;
    private final Timer timer = new Timer();

    public EvilIntakePiece(EvilIntake evilIntake, EvilIntakePosition evilPos){
        this.evilIntake = evilIntake;
        this.evilTarget = evilPos;

        addRequirements(evilIntake);
    }

    @Override
    public void initialize() {
        evilIntake.evilyummy(evilTarget);
        timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.hasElapsed(kSpinDelaySeconds)) {
            evilIntake.evileryummy(-1);
        }
    }

    // Called once the command ends or is interrupted.
     @Override
    public void end(boolean interrupted) {
        evilIntake.evilyummy(EvilIntakePosition.in);
        evilIntake.evileryummy(0);
    } 

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }
    
}