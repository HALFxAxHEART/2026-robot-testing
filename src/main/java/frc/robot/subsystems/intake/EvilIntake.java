package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.EvilIntakePosition;

public class EvilIntake extends SubsystemBase {
     private final EvilIntakeIO io;
     private final EvilIntakeIOInputsAutoLogged inputs = new EvilIntakeIOInputsAutoLogged();

     //debugging
     boolean hitPoint = false;
     double hitPointValue;
     int intakevalueid = 0;

    /** Constructs an Intake
     * @param io The hardware IO implementation
     * @param intakeID Kept only for the "Intake ID" dashboard label
     */
    public EvilIntake(EvilIntakeIO io, int intakeID){
        this.io = io;
        intakevalueid = intakeID;
        // Constant for the life of the robot -- publish once here instead of every
        // periodic() cycle.
        SmartDashboard.putNumber("Intake ID", intakevalueid);
    }

    // Go-go Gadget Rotate (Makes Intake Rotate)
    public void evilyummy(EvilIntakePosition pos){ //pos is 0.36 comming in from file "Constants.java" through "RobotContainer.java" Through "EvilIntakePiece.java" to here.
        hitPoint = true;
        hitPointValue = pos.getAngle();

        io.setRotationPosition(pos.getAngle());
    }

    public void evileryummy(double speed){
        io.setSpinPercent(speed);
    }

    public Command evilestyummy(EvilIntakePosition pos){
        return run(() -> evilyummy(pos));
    }

    public double getAngle(){
        return inputs.rotorPositionRotations;
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EvilIntake", inputs);

        SmartDashboard.putBoolean("Intake HitPoint", hitPoint);
        SmartDashboard.putNumber("Intake HitPointValue", hitPointValue);
        // positionRotations is already published via Logger.processInputs above -- no
        // need to duplicate it through a second NT write.
    }
}
