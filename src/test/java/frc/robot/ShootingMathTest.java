package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.hood.Hood;
import org.junit.jupiter.api.Test;

class ShootingMathTest {

    @Test
    void shooterRPSAtZeroDistanceMatchesBaseTerm() {
        assertEquals(20.9, ShootingMath.shooterRPSForDistance(0.0), 1e-9);
    }

    @Test
    void shooterRPSIncreasesWithDistance() {
        assertEquals(23.266, ShootingMath.shooterRPSForDistance(2.0), 1e-9);
        assertTrue(ShootingMath.shooterRPSForDistance(5.0) > ShootingMath.shooterRPSForDistance(2.0));
    }

    @Test
    void hoodAngleIsZeroBelowMinimumRange() {
        assertEquals(0.0, ShootingMath.hoodAngleForDistance(1.0));
        assertEquals(0.0, ShootingMath.hoodAngleForDistance(2.19));
    }

    @Test
    void hoodAngleFollowsLinearFormulaAboveMinimumRange() {
        assertEquals(1 - 0.463 * 2.2, ShootingMath.hoodAngleForDistance(2.2), 1e-9);
        assertEquals(1 - 0.463 * 3.0, ShootingMath.hoodAngleForDistance(3.0), 1e-9);
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
