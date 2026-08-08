package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.turret.ShootOnTheMoveSolver.ShotParameters;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

class ShootOnTheMoveSolverTest {

    @Test
    void zeroVelocityReturnsTheStaticShotUnchanged() {
        ShotParameters staticShot = new ShotParameters(10.0, Math.toRadians(50), Math.toRadians(30));
        ShotParameters result = ShootOnTheMoveSolver.solve(staticShot, new Translation2d(0, 0), 0.0);

        assertEquals(staticShot.speedMps(), result.speedMps(), 1e-9);
        assertEquals(staticShot.elevationRad(), result.elevationRad(), 1e-9);
        assertEquals(staticShot.azimuthRad(), result.azimuthRad(), 1e-9);
    }

    @Test
    void zeroVelocitySubtractsOnlyRobotHeadingFromAzimuth() {
        ShotParameters staticShot = new ShotParameters(10.0, Math.toRadians(50), Math.toRadians(30));
        ShotParameters result = ShootOnTheMoveSolver.solve(staticShot, new Translation2d(0, 0), Math.toRadians(10));

        assertEquals(Math.toRadians(20), result.azimuthRad(), 1e-9);
    }

    static Stream<Translation2d> shooterVelocities() {
        return Stream.of(
            new Translation2d(0, 0),
            new Translation2d(2.0, 0),
            new Translation2d(0, -1.5),
            new Translation2d(-3.0, 4.0),
            new Translation2d(6.0, 6.0)
        );
    }

    /**
     * Core correctness check: whatever shot the solver says to fire, once the shooter's
     * own velocity is added back on, the resulting ball velocity must exactly match the
     * static (stationary-robot) shot we asked it to reproduce. This is the physical
     * invariant the whole algorithm rests on -- if this doesn't hold, the equations are wrong.
     */
    @ParameterizedTest
    @MethodSource("shooterVelocities")
    void addingShooterVelocityBackReproducesTheStaticShot(Translation2d shooterVelocity) {
        ShotParameters staticShot = new ShotParameters(12.0, Math.toRadians(55), Math.toRadians(40));
        double robotHeading = Math.toRadians(15);

        ShotParameters shot = ShootOnTheMoveSolver.solve(staticShot, shooterVelocity, robotHeading);

        // Reconstruct the shot's field-frame velocity vector (azimuth was expressed in the
        // robot frame by solve(), so add the heading back to get to the field frame).
        double azimuthField = shot.azimuthRad() + robotHeading;
        double horizontal = shot.speedMps() * Math.cos(shot.elevationRad());
        double ballVx = shooterVelocity.getX() + horizontal * Math.cos(azimuthField);
        double ballVy = shooterVelocity.getY() + horizontal * Math.sin(azimuthField);
        double ballVz = shot.speedMps() * Math.sin(shot.elevationRad());

        double expectedHorizontal = staticShot.speedMps() * Math.cos(staticShot.elevationRad());
        double expectedVx = expectedHorizontal * Math.cos(staticShot.azimuthRad());
        double expectedVy = expectedHorizontal * Math.sin(staticShot.azimuthRad());
        double expectedVz = staticShot.speedMps() * Math.sin(staticShot.elevationRad());

        assertEquals(expectedVx, ballVx, 1e-9);
        assertEquals(expectedVy, ballVy, 1e-9);
        assertEquals(expectedVz, ballVz, 1e-9);
    }

    @Test
    void movingTowardTheTargetRequiresALowerShotSpeed() {
        // Static shot straight down +X. Shooter moving toward the target (+X) should need
        // less added speed than a stationary shooter to reach the same field-frame speed.
        ShotParameters staticShot = new ShotParameters(10.0, Math.toRadians(45), 0.0);

        ShotParameters stationary = ShootOnTheMoveSolver.solve(staticShot, new Translation2d(0, 0), 0.0);
        ShotParameters movingToward = ShootOnTheMoveSolver.solve(staticShot, new Translation2d(3.0, 0), 0.0);

        assertEquals(stationary.azimuthRad(), movingToward.azimuthRad(), 1e-9); // straight-on motion doesn't change azimuth
        org.junit.jupiter.api.Assertions.assertTrue(movingToward.speedMps() < stationary.speedMps());
    }

    @Test
    void shooterVelocityAtRobotCenterIgnoresRotation() {
        Translation2d v = ShootOnTheMoveSolver.shooterVelocity(
            new Translation2d(1.0, 2.0), 5.0, new Translation2d(0, 0));
        assertEquals(1.0, v.getX(), 1e-9);
        assertEquals(2.0, v.getY(), 1e-9);
    }

    @Test
    void shooterVelocityAddsTangentialComponentFromRotation() {
        // Turret 1m in front of a robot spinning at 2 rad/s with zero translational
        // velocity: tangential velocity should be (0, +2) -- perpendicular to the offset,
        // in the direction of rotation.
        Translation2d v = ShootOnTheMoveSolver.shooterVelocity(
            new Translation2d(0, 0), 2.0, new Translation2d(1.0, 0.0));
        assertEquals(0.0, v.getX(), 1e-9);
        assertEquals(2.0, v.getY(), 1e-9);
    }
}
