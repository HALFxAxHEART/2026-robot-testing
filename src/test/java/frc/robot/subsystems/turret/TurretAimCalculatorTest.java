package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

/**
 * Tests for the pure aim-solving math extracted from Turret.periodic(). These mirror
 * the real robot constants so a regression here means real robot behavior changed.
 */
class TurretAimCalculatorTest {

    // Same numbers Turret.java configures in production.
    private static final TurretAimCalculator.Config CONFIG = new TurretAimCalculator.Config(
        new Translation2d(Units.inchesToMeters(-5), Units.inchesToMeters(-6)), // turret offset
        new Translation2d(0.0, 4.105),   // blue target center
        new Translation2d(16.54, 4.105), // red target center
        new Translation2d(0.0, 0.0),     // target correction offset
        Rotation2d.fromDegrees(0.0),     // turret zero offset
        -1.0,                            // direction multiplier
        0.30,                            // max turret rotations
        200.0 / 16.0,                    // gear ratio
        16.54,                           // field length
        8.21,                            // field width
        1.5,                             // danger zone clearance
        6.0                              // estimated shot speed mps
    );

    private static final ChassisSpeeds STATIONARY = new ChassisSpeeds(0, 0, 0);

    @Test
    void blueOwnSideIsShootingMode() {
        Pose2d pose = new Pose2d(2.0, 4.105, Rotation2d.kZero);
        var solution = TurretAimCalculator.solve(CONFIG, pose, false, STATIONARY);
        assertEquals(Turret.Mode.SHOOTING, solution.mode());
    }

    @Test
    void blueOpponentSideIsPassingMode() {
        Pose2d pose = new Pose2d(10.0, 4.105, Rotation2d.kZero);
        var solution = TurretAimCalculator.solve(CONFIG, pose, false, STATIONARY);
        assertEquals(Turret.Mode.PASSING, solution.mode());
    }

    @Test
    void redOwnSideIsShootingMode() {
        Pose2d pose = new Pose2d(14.0, 4.105, Rotation2d.kZero);
        var solution = TurretAimCalculator.solve(CONFIG, pose, true, STATIONARY);
        assertEquals(Turret.Mode.SHOOTING, solution.mode());
    }

    @Test
    void redOpponentSideIsPassingMode() {
        Pose2d pose = new Pose2d(4.0, 4.105, Rotation2d.kZero);
        var solution = TurretAimCalculator.solve(CONFIG, pose, true, STATIONARY);
        assertEquals(Turret.Mode.PASSING, solution.mode());
    }

    /**
     * Robot near the hub but facing almost directly away from it -- the raw required
     * turn is ~0.49 rotations, comfortably past the 0.30 soft limit, so the result
     * should land exactly on the clamp boundary (-0.30 * gearRatio) regardless of the
     * exact unclamped angle. Guards the wrap-limit protection that keeps turret wiring
     * from winding up.
     */
    @Test
    void shootingSetpointClampsToMaxTurretRotations() {
        Pose2d pose = new Pose2d(2.105, 4.105, Rotation2d.kZero);
        var solution = TurretAimCalculator.solve(CONFIG, pose, false, STATIONARY);

        assertEquals(Turret.Mode.SHOOTING, solution.mode());
        double expectedClampedMotorRotations = -0.30 * CONFIG.turretGearRatio();
        assertEquals(expectedClampedMotorRotations, solution.targetMotorRotations(), 1e-9);
    }

    @Test
    void passTargetYSnapsAwayFromHubWhenRobotIsInTheDangerZone() {
        // Robot sitting right on the hub's Y centerline -- passing straight across would
        // cross in front of the hub, so the target should snap to the "above" side.
        Pose2d pose = new Pose2d(10.0, 4.105, Rotation2d.kZero);
        var solution = TurretAimCalculator.solve(CONFIG, pose, false, STATIONARY);

        assertEquals(Turret.Mode.PASSING, solution.mode());
        assertEquals(4.105 + 1.5, solution.passTargetY(), 1e-9);
    }

    @Test
    void passTargetYSnapsToOtherSideWhenBelowHubCenter() {
        Pose2d pose = new Pose2d(10.0, 4.0, Rotation2d.kZero);
        var solution = TurretAimCalculator.solve(CONFIG, pose, false, STATIONARY);

        assertEquals(4.105 - 1.5, solution.passTargetY(), 1e-9);
    }

    @Test
    void passTargetYIsUnchangedWhenFarFromDangerZone() {
        Pose2d pose = new Pose2d(10.0, 0.2, Rotation2d.kZero);
        var solution = TurretAimCalculator.solve(CONFIG, pose, false, STATIONARY);

        // 0.2 is outside the danger zone (|0.2 - 4.105| = 3.905 >= 1.5) but still gets
        // clamped into the [0.5, fieldWidth - 0.5] field-edge safety margin.
        assertEquals(0.5, solution.passTargetY(), 1e-9);
    }

    @Test
    void shootingModeReportsZeroForPassingFields() {
        Pose2d pose = new Pose2d(2.0, 4.105, Rotation2d.kZero);
        var solution = TurretAimCalculator.solve(CONFIG, pose, false, STATIONARY);

        assertEquals(0.0, solution.distanceToPassTargetMeters());
        assertEquals(0.0, solution.passTargetY());
    }

    @Test
    void passingModeReportsZeroForShootingFields() {
        Pose2d pose = new Pose2d(10.0, 4.105, Rotation2d.kZero);
        var solution = TurretAimCalculator.solve(CONFIG, pose, false, STATIONARY);

        assertEquals(0.0, solution.distanceToHubMeters());
        assertEquals(0.0, solution.virtualDistanceToHubMeters());
    }

    /**
     * Regression test for the turret-rotation lead correction (see
     * ShootOnTheMoveSolver.shooterVelocity, called from solveShooting): a turret offset
     * from the robot's center picks up tangential velocity purely from spinning, even
     * with zero translational velocity. Before that fix, omega was ignored entirely, so
     * this would have failed (both distances would be exactly equal).
     */
    @Test
    void spinningRobotShiftsLeadEvenWithZeroTranslationalVelocity() {
        Pose2d pose = new Pose2d(2.105, 4.105, Rotation2d.kZero);
        ChassisSpeeds spinning = new ChassisSpeeds(0, 0, 3.0);

        var stationary = TurretAimCalculator.solve(CONFIG, pose, false, STATIONARY);
        var spun = TurretAimCalculator.solve(CONFIG, pose, false, spinning);

        // Zero net turret velocity (no translation, no spin) means no lead correction at all.
        assertEquals(stationary.distanceToHubMeters(), stationary.virtualDistanceToHubMeters(), 1e-9);

        // Spinning alone shifts the lead-corrected distance away from the raw distance.
        assertTrue(Math.abs(spun.virtualDistanceToHubMeters() - spun.distanceToHubMeters()) > 1e-6);
    }

    @Test
    void motorRotationsNeverExceedGearRatioTimesMaxTurretRotations() {
        // Sweep a bunch of headings and confirm the clamp always holds, on both alliances.
        double limit = CONFIG.maxTurretRotations() * CONFIG.turretGearRatio();
        for (int deg = 0; deg < 360; deg += 15) {
            Pose2d pose = new Pose2d(2.0, 4.105, Rotation2d.fromDegrees(deg));
            var blue = TurretAimCalculator.solve(CONFIG, pose, false, STATIONARY);
            assertTrue(Math.abs(blue.targetMotorRotations()) <= limit + 1e-9,
                "blue heading " + deg + " exceeded clamp: " + blue.targetMotorRotations());

            Pose2d redPose = new Pose2d(14.0, 4.105, Rotation2d.fromDegrees(deg));
            var red = TurretAimCalculator.solve(CONFIG, redPose, true, STATIONARY);
            assertTrue(Math.abs(red.targetMotorRotations()) <= limit + 1e-9,
                "red heading " + deg + " exceeded clamp: " + red.targetMotorRotations());
        }
    }
}
