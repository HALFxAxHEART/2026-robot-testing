package frc.robot.subsystems.shooter;

/**
 * Converts flywheel motor RPM to theoretical ball exit speed, using this robot's actual
 * shooter geometry: a 4in "bottom" wheel driven 1:1 off the motor, and a 2in "top" wheel
 * driven off the bottom wheel at a 2:1 step-up.
 *
 * <p>That gearing is exactly what's needed to cancel the diameter difference and match
 * surface speeds (verified in {@code FlywheelPhysicsTest}): the bottom wheel's surface
 * speed is proportional to {@code diameter x RPM = 4 x N}, and the top wheel's is
 * {@code 2 x 2N = 4N} -- identical. That's the physical basis for "almost no spin on the
 * ball" -- it's not an approximation, the design cancels it exactly.
 *
 * <p>Speed formula: exit speed equals either wheel's (matched) surface speed, i.e. the
 * bottom wheel's circumference times its RPS, times an efficiency factor for slippage in
 * the 1/2" compression. See
 * https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805
 * (equation 2) for the general form this specializes.
 */
public final class FlywheelPhysics {

    /** Bottom wheel diameter, driven 1:1 off the shooter motor. */
    public static final double kBottomWheelDiameterInches = 4.0;

    /** Top wheel diameter, driven off the bottom wheel at a 2:1 step-up. */
    public static final double kTopWheelDiameterInches = 2.0;

    /** Bottom-wheel-to-top-wheel speed ratio (top spins this many times faster). */
    public static final double kTopWheelSpeedRatio = 2.0;

    /**
     * Fraction of theoretical surface speed actually imparted to the ball, accounting for
     * compression/slippage losses. This is FRC 3061 (Huskie Robotics)'s empirically
     * measured value for THEIR shooter (via slo-mo video), used here only as a starting
     * placeholder -- it is NOT measured for this robot's wheels/compression/game piece.
     * Replace with a real value once you've done the same slo-mo calibration they describe.
     */
    public static final double kAssumedEfficiency = 0.90;

    private FlywheelPhysics() {}

    /** @return Theoretical ball exit speed in ft/s, using {@link #kAssumedEfficiency}. */
    public static double exitSpeedFtPerSec(double bottomWheelMotorRPM) {
        return exitSpeedFtPerSec(bottomWheelMotorRPM, kAssumedEfficiency);
    }

    /**
     * @param bottomWheelMotorRPM The shooter motor's RPM -- this is the bottom wheel's RPM
     *                            directly, since that stage is a 1:1 direct drive (i.e. the
     *                            same value {@code leadShoot.getVelocity()} reads in
     *                            {@code ShooterIOTalonFX}, times 60 to go RPS -> RPM).
     * @param efficiency          Fraction of theoretical surface speed actually reaching the
     *                            ball (1.0 = no slippage). Pass your own measured value once
     *                            you have one instead of relying on the placeholder.
     * @return Theoretical ball exit speed in ft/s.
     */
    public static double exitSpeedFtPerSec(double bottomWheelMotorRPM, double efficiency) {
        double circumferenceFeet = (Math.PI * kBottomWheelDiameterInches) / 12.0;
        double revolutionsPerSecond = bottomWheelMotorRPM / 60.0;
        return circumferenceFeet * revolutionsPerSecond * efficiency;
    }

    /** @return Theoretical ball exit speed in m/s, using {@link #kAssumedEfficiency}. */
    public static double exitSpeedMetersPerSecond(double bottomWheelMotorRPM) {
        return exitSpeedMetersPerSecond(bottomWheelMotorRPM, kAssumedEfficiency);
    }

    /** @return Theoretical ball exit speed in m/s. See {@link #exitSpeedFtPerSec(double, double)}. */
    public static double exitSpeedMetersPerSecond(double bottomWheelMotorRPM, double efficiency) {
        return exitSpeedFtPerSec(bottomWheelMotorRPM, efficiency) * 0.3048;
    }

    /**
     * @return The top wheel's RPM for a given bottom wheel (motor) RPM, per the fixed 2:1
     *         mechanical step-up. Not independently commandable -- exposed for verifying
     *         the surface-speed-matching design, not for control code.
     */
    public static double topWheelRpmFor(double bottomWheelMotorRPM) {
        return bottomWheelMotorRPM * kTopWheelSpeedRatio;
    }
}
