package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSystem extends SubsystemBase {

    private final RollerSystemIO io;
    private final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();

    public RollerSystem(RollerSystemIO io){
        this.io = io;
    }

    // Sets speed using PID
    public void roll(double rollerSpeed){
        io.setVelocity(rollerSpeed);
    }

    // Cuts power to 0 volts instead of fighting PID, stopping the jitter!
    public void rollerStop(){
        io.setVoltage(0);
    }

     @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("RollerSystem", inputs);
    }
}
