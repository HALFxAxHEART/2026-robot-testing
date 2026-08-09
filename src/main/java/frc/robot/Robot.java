// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TorqueNados - FRC 5090

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

      private final boolean kUseLimelight = true;

    // Log and replay timestamp and joystick data.
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    @Override
    public void robotInit(){
     /*  if (RobotController.getUserButton()) {
            m_robotContainer.intake.intakeCoast();
        } else {
            m_robotContainer.intake.intakeBrake();
        } */
    }

    public Robot() {
        // AdvantageKit setup -- must happen before any other robot code runs (including
        // RobotContainer's subsystem construction), so this stays first in the constructor.
        Logger.recordMetadata("ProjectName", "2026-robot-testing");
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);

        switch (Constants.currentMode) {
            case REAL -> {
                // Log to a USB stick ("/U/logs") and publish live to NetworkTables/AdvantageScope.
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }
            case SIM -> {
                // Normal desktop simulation -- just publish live, no log file required to run.
                Logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                // Replaying a previously recorded log through the current code.
                setUseTiming(false); // run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }

        Logger.start(); // No more data receivers, replay sources, or metadata values may be added after this.

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    /* This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation of how to use vision should be tuned per-robot and to the team's specification. */
    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();

      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);



      Limelighthelpers.SetRobotOrientation("limelight",m_robotContainer.drivetrain.getgyroyaw().getDegrees(), 0, 0, 0, 0, 0);
      var llMeasurement = Limelighthelpers.getBotPoseEstimate_wpiBlue("limelight");
      Logger.recordOutput("Vision/Limelight/HasMeasurement", llMeasurement != null && llMeasurement.tagCount > 0);
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        Logger.recordOutput("Vision/Limelight/AppliedPose", llMeasurement.pose);
        Logger.recordOutput("Vision/Limelight/TagCount", llMeasurement.tagCount);

        Limelighthelpers.SetRobotOrientation("limelight-left",m_robotContainer.drivetrain.getgyroyaw().getDegrees(), 0, 0, 0, 0, 0);
        var llMeasurementleft = Limelighthelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        Logger.recordOutput("Vision/LimelightLeft/HasMeasurement", llMeasurementleft != null && llMeasurementleft.tagCount > 0);
        if (llMeasurementleft != null && llMeasurementleft.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
          m_robotContainer.drivetrain.addVisionMeasurement(llMeasurementleft.pose, llMeasurementleft.timestampSeconds);
          Logger.recordOutput("Vision/LimelightLeft/AppliedPose", llMeasurementleft.pose);
          Logger.recordOutput("Vision/LimelightLeft/TagCount", llMeasurementleft.tagCount);
        }
    }
}
  }

    /* Used to be used for the intializing of the robot when disabled. No idea why it was commented out.
     * @Override
     * public void disabledInit(){} */

    @Override
    public void disabledPeriodic(){}

    @Override
    public void disabledExit(){}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic(){}

    @Override
    public void autonomousExit(){}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
            // autonCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic(){}

    @Override
    public void teleopExit(){}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic(){}

    @Override
    public void testExit(){}

    @Override
    public void simulationPeriodic(){}
}
