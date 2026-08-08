package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class FlywheelPhysicsTest {

    private static double wheelSurfaceSpeedFtPerSec(double diameterInches, double rpm) {
        double circumferenceFeet = (Math.PI * diameterInches) / 12.0;
        return circumferenceFeet * (rpm / 60.0);
    }

    /**
     * The core mechanical claim: a 4in wheel direct-driven at RPM N and a 2in wheel driven
     * off it at 2N (per the real 2:1 step-up) must have identical surface speeds -- that's
     * the physical basis for "almost no spin on the ball" being an exact design property,
     * not an approximation. If this ever fails, either the gear ratio or wheel sizes in
     * FlywheelPhysics no longer match the real robot.
     */
    @Test
    void bottomAndTopWheelSurfaceSpeedsMatchExactlyByDesign() {
        double motorRpm = 3000.0;
        double bottomSurfaceSpeed = wheelSurfaceSpeedFtPerSec(FlywheelPhysics.kBottomWheelDiameterInches, motorRpm);
        double topSurfaceSpeed = wheelSurfaceSpeedFtPerSec(
            FlywheelPhysics.kTopWheelDiameterInches, FlywheelPhysics.topWheelRpmFor(motorRpm));

        assertEquals(bottomSurfaceSpeed, topSurfaceSpeed, 1e-9);
    }

    @Test
    void topWheelSpinsTwiceAsFastAsBottomWheel() {
        assertEquals(6000.0, FlywheelPhysics.topWheelRpmFor(3000.0), 1e-9);
    }

    @Test
    void zeroRpmIsZeroSpeed() {
        assertEquals(0.0, FlywheelPhysics.exitSpeedFtPerSec(0.0));
    }

    @Test
    void exitSpeedAtFullEfficiencyMatchesBottomWheelSurfaceSpeed() {
        double motorRpm = 3000.0;
        double expected = wheelSurfaceSpeedFtPerSec(FlywheelPhysics.kBottomWheelDiameterInches, motorRpm);

        assertEquals(expected, FlywheelPhysics.exitSpeedFtPerSec(motorRpm, 1.0), 1e-9);
    }

    @Test
    void exitSpeedScalesLinearlyWithEfficiency() {
        double motorRpm = 3000.0;
        double fullEfficiency = FlywheelPhysics.exitSpeedFtPerSec(motorRpm, 1.0);
        double halfEfficiency = FlywheelPhysics.exitSpeedFtPerSec(motorRpm, 0.5);

        assertEquals(fullEfficiency / 2.0, halfEfficiency, 1e-9);
    }

    @Test
    void metersPerSecondMatchesFeetPerSecondConversion() {
        double ftPerSec = FlywheelPhysics.exitSpeedFtPerSec(3000.0);
        double mPerSec = FlywheelPhysics.exitSpeedMetersPerSecond(3000.0);

        assertEquals(ftPerSec * 0.3048, mPerSec, 1e-9);
    }
}
