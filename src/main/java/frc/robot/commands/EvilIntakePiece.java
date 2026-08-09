package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EvilIntakePosition;
import frc.robot.subsystems.intake.EvilIntake;

/**
 * Deploys the intake to evilPos and keeps the rollers spinning the entire time this command
 * is active -- no delay before spinning up anymore, because a piece can jam the mechanism if
 * the rollers are idle while the arm is still swinging through it (EvilIntake.periodic()
 * also spins the rollers automatically during transit for the same reason, so this really
 * just covers the "already arrived, keep intaking" portion).
 */
public class EvilIntakePiece extends Command{
    EvilIntake evilIntake;
    EvilIntakePosition evilTarget;

    public EvilIntakePiece(EvilIntake evilIntake, EvilIntakePosition evilPos){
        this.evilIntake = evilIntake;
        this.evilTarget = evilPos;

        addRequirements(evilIntake);
    }

    @Override
    public void initialize() {
        evilIntake.evilyummy(evilTarget);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        evilIntake.evileryummy(-1);
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