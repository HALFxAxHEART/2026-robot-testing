package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class PassingMathTest {

    @Test
    void rpsAtNearSeedPointMatchesSeededValue() {
        assertEquals(45.0, PassingMath.shooterRPSForDistance(8.27), 1e-9);
    }

    @Test
    void rpsAtFarSeedPointMatchesSeededValue() {
        assertEquals(60.0, PassingMath.shooterRPSForDistance(16.54), 1e-9);
    }

    @Test
    void rpsInterpolatesLinearlyBetweenSeedPoints() {
        double midDistance = (8.27 + 16.54) / 2.0;
        assertEquals((45.0 + 60.0) / 2.0, PassingMath.shooterRPSForDistance(midDistance), 1e-9);
    }

    @Test
    void rpsClampsToNearSeedPointBelowIt() {
        // InterpolatingDoubleTreeMap holds the edge value rather than extrapolating.
        assertEquals(45.0, PassingMath.shooterRPSForDistance(1.0), 1e-9);
    }

    @Test
    void rpsClampsToFarSeedPointBeyondIt() {
        assertEquals(60.0, PassingMath.shooterRPSForDistance(20.0), 1e-9);
    }

    @Test
    void hoodAngleIsFlatAcrossBothSeedPoints() {
        assertEquals(-2.2, PassingMath.hoodAngleForDistance(8.27), 1e-9);
        assertEquals(-2.2, PassingMath.hoodAngleForDistance(16.54), 1e-9);
        assertEquals(-2.2, PassingMath.hoodAngleForDistance(12.0), 1e-9);
    }
}
