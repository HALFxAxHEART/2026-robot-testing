package frc.robot.subsystems.turret;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

// WPILib Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    /** What the turret is currently aiming at. Read this instead of the SmartDashboard string. */
    public enum Mode {
        SHOOTING,
        PASSING
    }
    private Mode m_mode = Mode.SHOOTING;

    // --- Hardware & Control ---
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    // --- External Dependencies ---
    private final Supplier<Pose2d> m_robotPoseSupplier;
    private final Supplier<ChassisSpeeds> m_robotVelocitySupplier;

    // --- ABSOLUTE FIELD TARGETS (THE GRID/HUB) ---
    private final Translation2d kBlueTargetCenter = new Translation2d(0.0, 4.105);
    private final Translation2d kRedTargetCenter = new Translation2d(16.54, 4.105);

    // --- PHYSICAL TURRET OFFSET ---
    private final double kTurretOffsetXInches = -5; // Backwards
    private final double kTurretOffsetYInches = -6;  // Right

    private final Translation2d m_robotRelativeTurretOffset = new Translation2d(
        Units.inchesToMeters(kTurretOffsetXInches),
        Units.inchesToMeters(kTurretOffsetYInches)
    );

    // NEW: Flips the direction if the turret is mirroring the target (turns left when target is right)
    private final double kTurretDirectionMultiplier = -1.0;

    // Adjust this until 0 motor rotations is perfectly facing backward.
    // Try 90.0, 180.0, or 270.0 now that the direction is fixed.
    private final Rotation2d kTurretZeroOffset = Rotation2d.fromDegrees(0.0);

    // --- TARGET OFFSET CORRECTION ---
    private final double kTargetCenterOffsetXInches = 0.0;
    private final double kTargetCenterOffsetYInches = 0.0;

    // --- Mechanical Constants ---
    private final double kTurretRingTeeth = 200.0;
    private final double kEncoderGearTeeth = 16.0;
    private final double kTurretGearRatio = kTurretRingTeeth / kEncoderGearTeeth;
    private final double kMaxTurretRotations = 0.30; //0.48

    // --- FIELD DIMENSIONS ---
    private static final double kFieldLengthMeters = 16.54;
    private static final double kFieldWidthMeters = 8.21;
    private static final double kDangerZoneClearanceMeters = 1.5;
    private static final double kEstimatedShotSpeedMPS = 6.0;

    private final TurretAimCalculator.Config m_aimConfig = new TurretAimCalculator.Config(
        m_robotRelativeTurretOffset,
        kBlueTargetCenter,
        kRedTargetCenter,
        new Translation2d(Units.inchesToMeters(kTargetCenterOffsetXInches), Units.inchesToMeters(kTargetCenterOffsetYInches)),
        kTurretZeroOffset,
        kTurretDirectionMultiplier,
        kMaxTurretRotations,
        kTurretGearRatio,
        kFieldLengthMeters,
        kFieldWidthMeters,
        kDangerZoneClearanceMeters,
        kEstimatedShotSpeedMPS
    );

    // --- LIVE STATE VARIABLES ---
    public double m_distanceToHubMeters = 0.0;
    public double m_distanceToPassTargetMeters = 0.0;
    public double m_virtualDistanceToHubMeters = 0.0;
    private double m_targetMotorRotations = 0.0;

    public Turret(TurretIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> velocitySupplier) {
        this.io = io;
        this.m_robotPoseSupplier = poseSupplier;
        this.m_robotVelocitySupplier = velocitySupplier;
    }

    public void zeroTurret() {
        io.setZeroPosition();
    }

    public double getDistanceToHubMeters() {
        return m_distanceToHubMeters;
    }

    public double getShootingDistance() {
        if (m_mode == Mode.PASSING) {
            return m_distanceToPassTargetMeters;
        }
        return m_virtualDistanceToHubMeters;
    }

    /** @return Whether the turret is currently in SHOOTING or PASSING mode, based on field zone. */
    public Mode getMode() {
        return m_mode;
    }

    public void alignToHub() {
        io.setMotionMagicPosition(m_targetMotorRotations);
    }

    public void goToZero() {
        io.setMotionMagicPosition(0);
    }

    public boolean isTurretReady(){
        if (m_targetMotorRotations == 0.0) {
            return false;
        }
        return Math.abs(inputs.positionMotorRotations - m_targetMotorRotations) <= 0.2;
    }

    public void passOrShoot() {
        io.setMotionMagicPosition(m_targetMotorRotations);
    }

    private boolean isRedAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    @Override
    public void periodic() {
        // --- LIVE MOTOR DATA ---
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        SmartDashboard.putNumber("Turret/Current_Motor_Rots", inputs.positionMotorRotations);
        SmartDashboard.putNumber("Turret/Current_Turret_Rots", inputs.positionMotorRotations / kTurretGearRatio);

        // --- SOLVE FOR THE AIM SOLUTION (pure math, see TurretAimCalculator) ---
        Pose2d robotPose = m_robotPoseSupplier.get();
        boolean isRed = isRedAlliance();
        ChassisSpeeds robotVelocity = m_robotVelocitySupplier.get();

        TurretAimCalculator.AimSolution solution =
            TurretAimCalculator.solve(m_aimConfig, robotPose, isRed, robotVelocity);

        m_mode = solution.mode();
        m_targetMotorRotations = solution.targetMotorRotations();
        m_distanceToHubMeters = solution.distanceToHubMeters();
        m_virtualDistanceToHubMeters = solution.virtualDistanceToHubMeters();
        m_distanceToPassTargetMeters = solution.distanceToPassTargetMeters();

        // --- TELEMETRY ---
        SmartDashboard.putString("Turret/Mode", m_mode.name());
        SmartDashboard.putNumber("Turret/Pass_Distance_Meters", m_distanceToPassTargetMeters);
        SmartDashboard.putNumber("Turret/Pass_Target_Y", solution.passTargetY());
        SmartDashboard.putNumber("Turret/Distance_To_Hub_Meters", m_distanceToHubMeters);
        SmartDashboard.putNumber("Turret/Virtual_Distance_Meters", m_virtualDistanceToHubMeters);
        SmartDashboard.putNumber("Turret/Target_Motor_Rots", m_targetMotorRotations);
        SmartDashboard.putNumber("Turret/Target_Turret_Rots", m_targetMotorRotations / kTurretGearRatio);
    }
}
