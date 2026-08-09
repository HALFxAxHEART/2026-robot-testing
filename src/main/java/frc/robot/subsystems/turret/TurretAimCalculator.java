package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Pure turret aim-solving math, pulled out of {@link Turret#periodic()} so it can be
 * unit tested without touching hardware (TalonFX, DriverStation, etc). Behavior here
 * must stay byte-for-byte equivalent to what Turret used to compute inline.
 */
public final class TurretAimCalculator {

    /** Fixed geometry/tuning inputs needed to solve for a turret aim solution. */
    public record Config(
        Translation2d turretOffsetFromRobot,
        Translation2d blueTargetCenter,
        Translation2d redTargetCenter,
        Translation2d targetCenterCorrectionOffset,
        Rotation2d turretZeroOffset,
        double turretDirectionMultiplier,
        double maxTurretRotations,
        double turretGearRatio,
        double fieldLengthMeters,
        double fieldWidthMeters,
        double dangerZoneClearanceMeters,
        double estimatedShotSpeedMps
    ) {}

    /** Everything Turret.periodic() needs to command hardware and publish telemetry. */
    public record AimSolution(
        Turret.Mode mode,
        double targetMotorRotations,
        double distanceToHubMeters,
        double virtualDistanceToHubMeters,
        double distanceToPassTargetMeters,
        double passTargetY
    ) {}

    private TurretAimCalculator() {}

    /**
     * @return The aim solution for the current pose/alliance/velocity. On your own half of
     * the field this shoots at the hub (SHOOTING); at midfield or in the opponent's half it
     * instead passes fuel back toward your own zone (PASSING), steering the pass around the
     * net near the hub so it doesn't fly into it.
     */
    public static AimSolution solve(Config cfg, Pose2d robotPose, boolean isRedAlliance, ChassisSpeeds robotVelocity) {
        double fieldMidpointX = cfg.fieldLengthMeters() / 2.0;
        double hubCenterY = cfg.fieldWidthMeters() / 2.0;

        boolean inOpponentOrMidZone = isRedAlliance
            ? (robotPose.getX() <= fieldMidpointX)
            : (robotPose.getX() >= fieldMidpointX);

        Translation2d fieldFrameTurretOffset = cfg.turretOffsetFromRobot().rotateBy(robotPose.getRotation());
        Translation2d globalTurretPos = robotPose.getTranslation().plus(fieldFrameTurretOffset);

        if (inOpponentOrMidZone) {
            return solvePassing(cfg, robotPose, isRedAlliance, globalTurretPos, hubCenterY);
        }
        return solveShooting(cfg, robotPose, isRedAlliance, robotVelocity, globalTurretPos, fieldFrameTurretOffset);
    }

    private static AimSolution solvePassing(
        Config cfg, Pose2d robotPose, boolean isRedAlliance, Translation2d globalTurretPos, double hubCenterY
    ) {
        double passTargetX = isRedAlliance ? cfg.fieldLengthMeters() : 0.0;
        double passTargetY = robotPose.getY();

        // If a straight pass at the robot's current Y would cross through the net around
        // the hub, shift the aim point to the nearer clear side instead -- this is what
        // keeps passes from flying into the net instead of over/around it.
        if (Math.abs(robotPose.getY() - hubCenterY) < cfg.dangerZoneClearanceMeters()) {
            passTargetY = (robotPose.getY() >= hubCenterY)
                ? hubCenterY + cfg.dangerZoneClearanceMeters()
                : hubCenterY - cfg.dangerZoneClearanceMeters();
        }
        passTargetY = MathUtil.clamp(passTargetY, 0.5, cfg.fieldWidthMeters() - 0.5);

        Translation2d passTarget = new Translation2d(passTargetX, passTargetY);
        Translation2d turretToPassTarget = passTarget.minus(globalTurretPos);
        double distanceToPassTargetMeters = turretToPassTarget.getNorm();

        double targetMotorRotations = solveTurretMotorRotations(cfg, turretToPassTarget, robotPose.getRotation());

        return new AimSolution(
            Turret.Mode.PASSING,
            targetMotorRotations,
            0.0,
            0.0,
            distanceToPassTargetMeters,
            passTargetY
        );
    }

    private static AimSolution solveShooting(
        Config cfg, Pose2d robotPose, boolean isRedAlliance, ChassisSpeeds robotVelocity,
        Translation2d globalTurretPos, Translation2d fieldFrameTurretOffset
    ) {
        Translation2d rawTargetTranslation = isRedAlliance ? cfg.redTargetCenter() : cfg.blueTargetCenter();
        Translation2d finalTargetTranslation = isRedAlliance
            ? rawTargetTranslation.plus(cfg.targetCenterCorrectionOffset())
            : rawTargetTranslation.minus(cfg.targetCenterCorrectionOffset());

        Translation2d turretToTarget = finalTargetTranslation.minus(globalTurretPos);
        double distanceToHubMeters = turretToTarget.getNorm();

        // Velocity the turret itself is moving at -- not just the robot's center velocity.
        // A turret offset from the center of rotation picks up extra tangential velocity
        // whenever the robot is turning (v = v_center + omega x r_offset), which matters
        // for the lead calculation below. See "Accounting for Robot Rotation" in
        // https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805
        Translation2d turretVelocity = ShootOnTheMoveSolver.shooterVelocity(
            new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond),
            robotVelocity.omegaRadiansPerSecond,
            fieldFrameTurretOffset
        );

        double timeOfFlight = distanceToHubMeters / cfg.estimatedShotSpeedMps();
        Translation2d inheritedVelocityOffset = turretVelocity.times(timeOfFlight);
        Translation2d virtualTargetTranslation = finalTargetTranslation.minus(inheritedVelocityOffset);
        Translation2d turretToVirtualTarget = virtualTargetTranslation.minus(globalTurretPos);
        double virtualDistanceToHubMeters = turretToVirtualTarget.getNorm();

        double targetMotorRotations = solveTurretMotorRotations(cfg, turretToVirtualTarget, robotPose.getRotation());

        return new AimSolution(
            Turret.Mode.SHOOTING,
            targetMotorRotations,
            distanceToHubMeters,
            virtualDistanceToHubMeters,
            0.0,
            0.0
        );
    }

    /** Shared "point the turret at this field-relative vector" math used by both modes. */
    private static double solveTurretMotorRotations(Config cfg, Translation2d turretToTarget, Rotation2d robotRotation) {
        Rotation2d turretSetpoint = turretToTarget.getAngle()
            .minus(robotRotation)
            .minus(cfg.turretZeroOffset());
        double desiredTurretRotations = turretSetpoint.getRadians() / (2 * Math.PI);

        desiredTurretRotations *= cfg.turretDirectionMultiplier();

        desiredTurretRotations = Math.IEEEremainder(desiredTurretRotations, 1.0);
        desiredTurretRotations = MathUtil.clamp(desiredTurretRotations, -cfg.maxTurretRotations(), cfg.maxTurretRotations());

        return desiredTurretRotations * cfg.turretGearRatio();
    }
}
