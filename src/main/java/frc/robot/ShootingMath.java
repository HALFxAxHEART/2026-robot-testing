package frc.robot;

/**
 * Pure shooter-RPS / hood-angle formulas, pulled out of RobotContainer so they can be
 * unit tested without spinning up a whole RobotContainer (drivetrain, CAN devices, etc).
 * These cover the "New Equation 3/20/26" hub-shooting curves only -- passing-mode still
 * goes through the InterpolatingDoubleTreeMap lookups in RobotContainer.
 */
public final class ShootingMath {

    private ShootingMath() {}

    /** @return Flywheel setpoint in RPS for a given hub-shooting distance in meters. */
    public static double shooterRPSForDistance(double targetDistMeters) {
        return 20.9 + 0.697 * targetDistMeters + 0.243 * Math.pow(targetDistMeters, 2);
    }

    /** @return Hood setpoint (unclamped) for a given hub-shooting distance in meters. */
    public static double hoodAngleForDistance(double targetDistMeters) {
        if (targetDistMeters >= 2.2) {
            return 1 - (0.463 * targetDistMeters);
        }
        return 0;
    }
}
