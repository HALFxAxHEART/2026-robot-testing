package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

/**
 * Pure shooter-RPS / hood-angle formulas, pulled out of RobotContainer so they can be
 * unit tested without spinning up a whole RobotContainer (drivetrain, CAN devices, etc).
 * These cover the "New Equation 3/20/26" hub-shooting curves, plus a real-test-data
 * override -- passing-mode still goes through the InterpolatingDoubleTreeMap lookups in
 * RobotContainer.
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

    /**
     * Real shooter test data from "Turret Raw Values - Sheet1.csv" (in the repo root),
     * rows for Physical Distance 42-150 (in 6in steps), all taken with Hood Angle held
     * at 0. The sheet's "Cal" column (the calibrated RPS target -- confirmed against the
     * "actual RPM" column, which equals Cal*60 exactly on every row) is a perfectly
     * linear function of distance across all 19 points: every 6in step adds exactly
     * 0.4104 RPS. That's linear enough to use as a closed-form formula instead of an
     * interpolation table.
     *
     * <p>Confirmed: "Physical Distance" in the sheet is inches, and this whole range was
     * tested with a flat (0) hood -- angling the hood earlier wasn't needed and didn't
     * help, so {@link #hoodAngleForDistance} keeps the hood flat across this exact same
     * range (see {@link #kFlatHoodMaxDistanceMeters}) rather than starting to angle it
     * partway through, matching what was actually validated instead of extrapolating.
     */
    private static final double kMeasuredMinDistanceMeters = Units.inchesToMeters(42.0);
    private static final double kMeasuredMaxDistanceMeters = Units.inchesToMeters(150.0);

    /**
     * Hood stays flat (0) out to this distance, matching the real test data above --
     * confirmed with the team that angling the hood earlier wasn't needed. Deliberately
     * the same distance as {@link #kMeasuredMaxDistanceMeters}: beyond this point neither
     * the RPS curve nor the hood angle is backed by real test data anymore.
     */
    private static final double kFlatHoodMaxDistanceMeters = kMeasuredMaxDistanceMeters;
    private static final double kMeasuredRpsAtZeroInches = 19.6;
    private static final double kMeasuredRpsPerInch = 0.0684;

    private ShootingMath() {}

    /**
     * @return Flywheel setpoint in RPS for a given hub-shooting distance in meters, clamped
     *         to a sane ceiling. Uses the real measured/calibrated curve within the tested
     *         range (see {@link #kMeasuredMinDistanceMeters}), and the older quadratic
     *         formula outside it.
     */
    public static double shooterRPSForDistance(double targetDistMeters) {
        double rps;
        if (targetDistMeters >= kMeasuredMinDistanceMeters && targetDistMeters <= kMeasuredMaxDistanceMeters) {
            double distanceInches = Units.metersToInches(targetDistMeters);
            rps = kMeasuredRpsAtZeroInches + kMeasuredRpsPerInch * distanceInches;
        } else {
            rps = 20.9 + 0.697 * targetDistMeters + 0.243 * Math.pow(targetDistMeters, 2);
        }
        return MathUtil.clamp(rps, 0, kMaxShooterRPS);
    }

    /** @return Hood setpoint (unclamped) for a given hub-shooting distance in meters. */
    public static double hoodAngleForDistance(double targetDistMeters) {
        if (targetDistMeters >= kFlatHoodMaxDistanceMeters) {
            return 1 - (0.463 * targetDistMeters);
        }
        return 0;
    }
}
