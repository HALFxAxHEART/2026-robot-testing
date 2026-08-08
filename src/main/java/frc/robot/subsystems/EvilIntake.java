package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.EvilIntakePosition;

public class EvilIntake extends SubsystemBase {
     private final EvilIntakeIO io;
     private final EvilIntakeIOInputsAutoLogged inputs = new EvilIntakeIOInputsAutoLogged();
     EvilIntakePosition pos = EvilIntakePosition.in;

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
    }

        // Go-go Gadget Move (Makes the Intake Move)
    public void evilyummy(){
        //intakeMotor.set(1);
    }

    // Go-go Gadget Rotate (Makes Intake Rotate)
    public void evilyummy(EvilIntakePosition pos){ //pos is 0.35 comming in from file "Constants.java" through "RobotContainer.java" Through "EvilIntakePiece.java" to here.
        hitPoint = true;
        hitPointValue = pos.getAngle();

        //this.pos = pos;

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

        SmartDashboard.putNumber("Intake ID", intakevalueid);
        SmartDashboard.putBoolean("Intake HitPoint", hitPoint);
        SmartDashboard.putNumber("Intake HitPointValue", hitPointValue);
        SmartDashboard.putNumber("Intake Position", inputs.positionRotations);
        /* SmartDashboard.putNumber("Intake Position Degrees", getAngle());
        SmartDashboard.putString("Intake Target Position", pos.name());
        SmartDashboard.putNumber("Intake Target Revolutions", pos.getAngle());
        SmartDashboard.putNumber("Intake RPM", intakeMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("stator current", rotationMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("supply current", rotationMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("torque current", rotationMotor.getTorqueCurrent().getValueAsDouble()); */
    }
}
