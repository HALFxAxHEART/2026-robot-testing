package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Pure passing-mode RPS/hood lookups, pulled out of RobotContainer so they can be unit
 * tested without spinning up a whole RobotContainer (drivetrain, CAN devices, etc), mirroring
 * {@link ShootingMath}'s hub-shooting equivalent. Unlike ShootingMath's hub-shooting curve,
 * these two points are NOT measured test data -- just seeded placeholders (see the original
 * "Populate the passing maps with your field-length data" comment this replaced) -- so treat
 * PASSING-mode shots as unverified until someone tunes these against real pass distances.
 *
 * <p>Named "RPS" (not the original "Rpm") because the value flows straight into
 * {@link frc.robot.subsystems.shooter.Shooter#goShoot(double)}, which is rotations per
 * second -- same units mismatch this whole codebase's shooter test data turned out to have.
 */
public final class PassingMath {
    private static final InterpolatingDoubleTreeMap kRpsMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap kHoodMap = new InterpolatingDoubleTreeMap();

    static {
        // TODO: placeholder points, not measured pass data -- tune with real testing.
        kRpsMap.put(8.27, 45.0);
        kRpsMap.put(16.54, 60.0);
        kHoodMap.put(8.27, -2.2);
        kHoodMap.put(16.54, -2.2);
    }

    private PassingMath() {}

    /** @return Interpolated flywheel setpoint in RPS for a given passing distance in meters. */
    public static double shooterRPSForDistance(double targetDistMeters) {
        return kRpsMap.get(targetDistMeters);
    }

    /** @return Interpolated hood setpoint for a given passing distance in meters. */
    public static double hoodAngleForDistance(double targetDistMeters) {
        return kHoodMap.get(targetDistMeters);
    }
}
