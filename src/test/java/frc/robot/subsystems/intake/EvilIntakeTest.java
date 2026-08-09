package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.EvilIntakePosition;
import org.junit.jupiter.api.Test;

/** Covers the rack-and-pinion stall-detection/auto-retract behavior. */
class EvilIntakeTest {

    /** Fake IO that just echoes back the position/current a test sets on it. */
    private static class FakeEvilIntakeIO implements EvilIntakeIO {
        double positionRotations = EvilIntake.kReverseSoftLimitRotations;
        double statorCurrentAmps = 0.0;
        double lastCommandedPositionRotations = Double.NaN;

        @Override
        public void updateInputs(EvilIntakeIOInputs inputs) {
            inputs.positionRotations = positionRotations;
            inputs.rotorPositionRotations = positionRotations;
            inputs.statorCurrentAmps = statorCurrentAmps;
        }

        @Override
        public void setRotationPosition(double positionRotations) {
            lastCommandedPositionRotations = positionRotations;
        }
    }

    // --- isStallCondition(): pure logic, no Debouncer/real-time involved ---

    @Test
    void notStalledWhenCurrentIsNormal() {
        assertFalse(EvilIntake.isStallCondition(2.0, 5.0, EvilIntakePosition.out.getAngle()));
    }

    @Test
    void notStalledWhenAlreadyNearTargetEvenWithHighCurrent() {
        // High current while holding position (e.g. against a soft limit) isn't a stall.
        assertFalse(EvilIntake.isStallCondition(30.0, 16.8, EvilIntakePosition.out.getAngle()));
    }

    @Test
    void stalledWhenHighCurrentAndFarFromTarget() {
        assertTrue(EvilIntake.isStallCondition(30.0, 5.0, EvilIntakePosition.out.getAngle()));
    }

    // --- Full periodic()-driven behavior, through a fake IO. Uses real sleeps since the
    // stall detector is debounced against real elapsed time -- these run in well under a
    // second each. ---

    private static void runPeriodicUntil(EvilIntake intake, java.util.function.BooleanSupplier condition) throws InterruptedException {
        long deadline = System.currentTimeMillis() + 1000;
        while (System.currentTimeMillis() < deadline && !condition.getAsBoolean()) {
            intake.periodic();
            Thread.sleep(20);
        }
    }

    @Test
    void detectsStallAndRedirectsToInEvenIfACommandKeepsAskingForOut() throws InterruptedException {
        FakeEvilIntakeIO io = new FakeEvilIntakeIO();
        EvilIntake intake = new EvilIntake(io, 0);

        // Stuck partway out, drawing stall-level current -- as if it rammed a wall.
        io.positionRotations = 5.0;
        io.statorCurrentAmps = 30.0;
        intake.evilyummy(EvilIntakePosition.out);

        runPeriodicUntil(intake, intake::isStalled);
        assertTrue(intake.isStalled());
        assertEquals(EvilIntakePosition.in.getAngle(), io.lastCommandedPositionRotations, 1e-9);

        // Simulates evilestyummy()'s run(() -> evilyummy(pos)) re-calling evilyummy(out)
        // every scheduler cycle (as it does for the whole time an auto command like
        // DropIntake is active) -- this must NOT win back control while still stalled and
        // not yet actually home, or the fix in evilyummy() itself isn't doing its job.
        intake.evilyummy(EvilIntakePosition.out);
        assertEquals(EvilIntakePosition.in.getAngle(), io.lastCommandedPositionRotations, 1e-9,
            "evilyummy(out) should stay redirected to in while stalled");
    }

    @Test
    void clearsTheLatchOnceActuallyHomeAndAllowsExtendingAgain() throws InterruptedException {
        FakeEvilIntakeIO io = new FakeEvilIntakeIO();
        EvilIntake intake = new EvilIntake(io, 0);

        io.positionRotations = 5.0;
        io.statorCurrentAmps = 30.0;
        intake.evilyummy(EvilIntakePosition.out);
        runPeriodicUntil(intake, intake::isStalled);
        assertTrue(intake.isStalled());

        // Mechanism actually makes it back home and current drops back to normal.
        io.positionRotations = EvilIntakePosition.in.getAngle();
        io.statorCurrentAmps = 1.0;
        intake.periodic();
        assertFalse(intake.isStalled());

        // A fresh "out" request now goes through normally instead of staying redirected.
        intake.evilyummy(EvilIntakePosition.out);
        assertEquals(EvilIntakePosition.out.getAngle(), io.lastCommandedPositionRotations, 1e-9);
    }

    @Test
    void neverStallsWhenNothingHasCommandedAPositionYet() throws InterruptedException {
        FakeEvilIntakeIO io = new FakeEvilIntakeIO();
        EvilIntake intake = new EvilIntake(io, 0);

        // hitPointValue defaults to 0.0 (no command issued) -- high resting current here
        // shouldn't be possible on the real robot, but confirms periodic() doesn't false-
        // trigger before any real command exists.
        io.statorCurrentAmps = 30.0;
        Thread.sleep(300);
        intake.periodic();
        assertFalse(intake.isStalled());
    }
}
