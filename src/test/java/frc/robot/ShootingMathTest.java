package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hood.Hood;
import org.junit.jupiter.api.Test;

class ShootingMathTest {

    @Test
    void shooterRPSAtZeroDistanceMatchesBaseTerm() {
        assertEquals(20.9, ShootingMath.shooterRPSForDistance(0.0), 1e-9);
    }

    @Test
    void shooterRPSIncreasesWithDistance() {
        // Both points are below the measured-data range (42-150in), so this exercises the
        // older quadratic formula specifically.
        assertTrue(ShootingMath.shooterRPSForDistance(0.8) > ShootingMath.shooterRPSForDistance(0.3));
    }

    /**
     * These are transcribed directly from "Turret Raw Values - Sheet1.csv"'s Cal column
     * (Physical Distance 42/96/150, Hood Angle 0) -- not derived from the formula being
     * tested, so this actually checks the real data was wired in correctly.
     */
    @Test
    void shooterRPSUsesRealTestDataWithinTheTestedRange() {
        assertEquals(22.4728, ShootingMath.shooterRPSForDistance(Units.inchesToMeters(42.0)), 1e-6);
        assertEquals(26.1664, ShootingMath.shooterRPSForDistance(Units.inchesToMeters(96.0)), 1e-6);
        assertEquals(29.86, ShootingMath.shooterRPSForDistance(Units.inchesToMeters(150.0)), 1e-6);
    }

    @Test
    void shooterRPSFallsBackToTheOldFormulaOutsideTheTestedRange() {
        // Just below the tested range (42in): the measured line's formula would predict
        // 19.6 + 0.0684*41 = 22.4044, but since this is outside the tested window it
        // should fall back to the older quadratic instead.
        double justBelowRange = Units.inchesToMeters(41.0);
        double oldFormula = 20.9 + 0.697 * justBelowRange + 0.243 * Math.pow(justBelowRange, 2);

        assertEquals(oldFormula, ShootingMath.shooterRPSForDistance(justBelowRange), 1e-9);
    }

    @Test
    void shooterRPSClampsAtExtremeDistance() {
        // Field diagonal is ~18.5m; well beyond that the unclamped quadratic would exceed
        // 100 RPS. Confirms the sanity ceiling actually engages instead of growing forever.
        assertEquals(ShootingMath.kMaxShooterRPS, ShootingMath.shooterRPSForDistance(50.0), 1e-9);
    }

    /**
     * Hood stays flat all the way out to 150in (matching the real test data range and
     * confirmed with the team: angling it earlier wasn't needed and didn't help), not just
     * to the old 2.2m threshold.
     */
    @Test
    void hoodAngleIsZeroThroughTheWholeTestedRange() {
        assertEquals(0.0, ShootingMath.hoodAngleForDistance(1.0));
        assertEquals(0.0, ShootingMath.hoodAngleForDistance(2.2));
        assertEquals(0.0, ShootingMath.hoodAngleForDistance(3.0));
        assertEquals(0.0, ShootingMath.hoodAngleForDistance(Units.inchesToMeters(149.0)));
    }

    @Test
    void hoodAngleFollowsLinearFormulaBeyondTheTestedRange() {
        double threshold = Units.inchesToMeters(150.0);
        assertEquals(1 - 0.463 * threshold, ShootingMath.hoodAngleForDistance(threshold), 1e-9);
        assertEquals(1 - 0.463 * 4.0, ShootingMath.hoodAngleForDistance(4.0), 1e-9);
    }

    @Test
    void hoodSoftLimitsAreOrderedLowToHigh() {
        // Hood.kMaxExtensionRotations is the more-negative "physical max" bound and
        // kMinExtensionRotations is the resting position, so max < min numerically.
        // MathUtil.clamp(value, low, high) requires low <= high -- if someone ever
        // "fixes" these names without updating the call site, this test catches it.
        assertTrue(Hood.kMaxExtensionRotations < Hood.kMinExtensionRotations,
            "clamp bounds are inverted -- MathUtil.clamp(value, kMaxExtensionRotations, kMinExtensionRotations) would misbehave");
    }
}
