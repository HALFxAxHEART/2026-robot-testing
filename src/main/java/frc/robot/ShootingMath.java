package frc.robot;

import edu.wpi.first.math.MathUtil;

/**
 * Pure shooter-RPS / hood-angle formulas, pulled out of RobotContainer so they can be
 * unit tested without spinning up a whole RobotContainer (drivetrain, CAN devices, etc).
 * These cover the "New Equation 3/20/26" hub-shooting curves only -- passing-mode still
 * goes through the InterpolatingDoubleTreeMap lookups in RobotContainer.
 */
public final class ShootingMath {

    /**
     * Sanity ceiling on commanded flywheel speed. The quadratic formula below has no
     * built-in bound and grows with distance^2, so a large enough (e.g. lead-corrected)
     * virtual distance could otherwise command an arbitrarily high RPS. This value is a
     * generous placeholder -- comfortably above the ~43 RPS the formula produces at the
     * edge of the normal SHOOTING-mode zone -- not a measured max-safe-flywheel-speed.
     * Tighten it once you know the real limit.
     */
    public static final double kMaxShooterRPS = 70.0;

    private ShootingMath() {}

    /** @return Flywheel setpoint in RPS for a given hub-shooting distance in meters, clamped to a sane ceiling. */
    public static double shooterRPSForDistance(double targetDistMeters) {
        double rps = 20.9 + 0.697 * targetDistMeters + 0.243 * Math.pow(targetDistMeters, 2);
        return MathUtil.clamp(rps, 0, kMaxShooterRPS);
    }

    /** @return Hood setpoint (unclamped) for a given hub-shooting distance in meters. */
    public static double hoodAngleForDistance(double targetDistMeters) {
        if (targetDistMeters >= 2.2) {
            return 1 - (0.463 * targetDistMeters);
        }
        return 0;
    }
}
