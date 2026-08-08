package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Shoot-on-the-move via 3D velocity subtraction, as described by FRC 3061 (Huskie
 * Robotics) in "Huskie Physics: Shoot-on-the-move with Equations":
 * https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805
 *
 * <p>The idea: pick the shot you'd want to make from a <b>stationary</b> robot at this
 * field position -- a launch speed and direction, {@link ShotParameters}. A launched ball
 * doesn't "remember" whether the robot that fired it was moving, so if the robot is
 * moving, the ball inherits the robot's velocity on top of whatever the shooter gives it.
 * To make the ball still follow your chosen stationary-shot trajectory, subtract the
 * robot's velocity from the desired ball velocity <i>before</i> converting back to a
 * launch speed/angle -- that's the shot the moving robot actually needs to fire.
 *
 * <p>This is a different (more rigorous) approach than the "virtual target" trick in
 * {@link TurretAimCalculator}, which just shifts the aim point by
 * {@code robotVelocity * estimatedTimeOfFlight} and re-aims a static shot at the shifted
 * point. Velocity subtraction solves for the actual required shot speed, elevation, and
 * azimuth directly, so it doesn't rely on "shooting statically at a shifted point"
 * happening to approximate the true correction.
 *
 * <p><b>This class is not wired into the live Turret/Shooter/Hood control path.</b> Doing
 * so needs a real static shot map -- physical ball exit speed (m/s) and launch angle as
 * functions of distance, calibrated from the real shooter (the source post's section 2
 * describes the slo-mo-video calibration recipe: tape a reference ruler to the shooter,
 * film at 240fps, measure exit speed/angle for a handful of known shots). That's hardware
 * data this repo doesn't have. Once you have it, {@link #solve} is ready to use --
 * see docs/shoot-on-the-move.md for how to wire it in.
 */
public final class ShootOnTheMoveSolver {

    /**
     * A shot described by physical launch speed and direction, rather than motor units.
     *
     * @param speedMps      Ball launch speed in meters/second.
     * @param elevationRad  Angle above horizontal (0 = flat shot, PI/2 = straight up).
     * @param azimuthRad    Field-frame direction the shot travels in the horizontal plane
     *                      (standard atan2 convention: 0 = +X axis).
     */
    public record ShotParameters(double speedMps, double elevationRad, double azimuthRad) {}

    private ShootOnTheMoveSolver() {}

    /**
     * @param staticShot               The shot you'd make from a stationary robot at this
     *                                 field position (from your calibrated static shot map).
     * @param shooterVelocityFieldFrame Field-frame velocity of the point the ball actually
     *                                 leaves from (not necessarily the robot's center --
     *                                 see {@link #shooterVelocity} if the shooter is offset
     *                                 and the robot is rotating).
     * @param robotHeadingRad          Robot heading in the field frame (radians), used to
     *                                 convert the result's azimuth into the robot/turret
     *                                 frame, ready to command a turret setpoint from.
     * @return The shot the moving robot needs to fire so the ball follows the same
     *         field-frame trajectory as {@code staticShot}, with azimuth already
     *         expressed in the robot frame.
     */
    public static ShotParameters solve(
        ShotParameters staticShot,
        Translation2d shooterVelocityFieldFrame,
        double robotHeadingRad
    ) {
        double vStar = staticShot.speedMps();
        double horizontalStar = vStar * Math.cos(staticShot.elevationRad());
        double vxStar = horizontalStar * Math.cos(staticShot.azimuthRad());
        double vyStar = horizontalStar * Math.sin(staticShot.azimuthRad());
        double vzStar = vStar * Math.sin(staticShot.elevationRad());

        // The ball needs field-frame velocity (vxStar, vyStar, vzStar). It automatically
        // inherits the shooter's velocity, so the shot itself only needs to supply the
        // remainder. The robot moves purely horizontally, so it doesn't touch vz.
        double vxShot = vxStar - shooterVelocityFieldFrame.getX();
        double vyShot = vyStar - shooterVelocityFieldFrame.getY();
        double vzShot = vzStar;

        double horizontalShot = Math.hypot(vxShot, vyShot);
        double speedShot = Math.hypot(horizontalShot, vzShot);
        double elevationShot = Math.atan2(vzShot, horizontalShot);
        double azimuthFieldShot = Math.atan2(vyShot, vxShot);
        double azimuthRobotShot = wrapToPi(azimuthFieldShot - robotHeadingRad);

        return new ShotParameters(speedShot, elevationShot, azimuthRobotShot);
    }

    /**
     * @return The field-frame velocity of a point rigidly attached to the robot at
     *         {@code fieldFrameOffsetFromCenter} (e.g. a turret-mounted shooter), combining
     *         the robot's translational velocity with the tangential velocity added by its
     *         rotation: v = v_center + omega x r. This is the "less important, but simple to
     *         include" correction from the source post's section 2.3. {@link TurretAimCalculator}
     *         calls this directly for its own (already-live) shoot-on-the-move lead calculation.
     */
    public static Translation2d shooterVelocity(
        Translation2d robotVelocityFieldFrame,
        double robotOmegaRadPerSec,
        Translation2d fieldFrameOffsetFromCenter
    ) {
        double tangentialVelX = -fieldFrameOffsetFromCenter.getY() * robotOmegaRadPerSec;
        double tangentialVelY = fieldFrameOffsetFromCenter.getX() * robotOmegaRadPerSec;
        return new Translation2d(
            robotVelocityFieldFrame.getX() + tangentialVelX,
            robotVelocityFieldFrame.getY() + tangentialVelY
        );
    }

    private static double wrapToPi(double radians) {
        return Math.IEEEremainder(radians, 2 * Math.PI);
    }
}
