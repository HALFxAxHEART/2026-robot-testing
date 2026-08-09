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
        double lastCommandedSpinPercent = Double.NaN;

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

        @Override
        public void setSpinPercent(double percent) {
            lastCommandedSpinPercent = percent;
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

    @Test
    void stalledWorksInTheRetractingDirectionToo() {
        // Target (in) is LESS than current position -- a ball jammed partway through
        // retraction, not the extending-toward-a-wall case.
        assertTrue(EvilIntake.isStallCondition(30.0, 8.0, EvilIntakePosition.in.getAngle()));
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

    @Test
    void inDirectionStallDoesNotAutoRetractOrBlockGoingOutAfterward() throws InterruptedException {
        // A jammed game piece while retracting is NOT the same failure mode as hitting a
        // wall while extending -- EvilIntake must leave this one for FunnelAgitate to react
        // to, not silently force it back toward `in` (which is where it's already headed)
        // or block a subsequent evilyummy(out) call the way the wall-hit case does.
        FakeEvilIntakeIO io = new FakeEvilIntakeIO();
        EvilIntake intake = new EvilIntake(io, 0);

        io.positionRotations = 8.0;
        io.statorCurrentAmps = 30.0;
        intake.evilyummy(EvilIntakePosition.in);

        runPeriodicUntil(intake, intake::isStalled);
        assertTrue(intake.isStalled());

        // periodic() must NOT have redirected hitPointValue away from `in` on its own.
        assertEquals(EvilIntakePosition.in.getAngle(), io.lastCommandedPositionRotations, 1e-9);

        // A caller asking for `out` (e.g. FunnelAgitate backing off) must go through
        // unredirected -- this is exactly the case the wall-hit redirect must NOT catch.
        intake.evilyummy(EvilIntakePosition.out);
        assertEquals(EvilIntakePosition.out.getAngle(), io.lastCommandedPositionRotations, 1e-9);
    }

    @Test
    void spinsRollersWhileInTransitAndStopsForcingThemOnceArrived() {
        FakeEvilIntakeIO io = new FakeEvilIntakeIO();
        EvilIntake intake = new EvilIntake(io, 0);

        io.positionRotations = 5.0; // nowhere near `out` (17) yet
        intake.evilyummy(EvilIntakePosition.out);
        intake.periodic();
        assertTrue(io.lastCommandedSpinPercent < 0, "rollers should be spinning while in transit");

        io.lastCommandedSpinPercent = Double.NaN;
        io.positionRotations = EvilIntakePosition.out.getAngle(); // arrived
        intake.periodic();
        assertTrue(Double.isNaN(io.lastCommandedSpinPercent),
            "periodic() shouldn't touch the rollers once arrived -- that's the active command's job");
    }
}
