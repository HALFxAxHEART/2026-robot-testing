package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.EvilIntakePosition;
import frc.robot.subsystems.intake.EvilIntake;
import frc.robot.subsystems.intake.EvilIntakeIO;
import org.junit.jupiter.api.Test;

class FunnelAgitateTest {

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

    @Test
    void doesNothingWhenNotSupposedToAgitate() {
        FakeEvilIntakeIO io = new FakeEvilIntakeIO();
        EvilIntake intake = new EvilIntake(io, 0);
        FunnelAgitate agitate = new FunnelAgitate(intake, () -> false);

        agitate.execute();
        assertTrue(Double.isNaN(io.lastCommandedPositionRotations), "shouldn't command anything when not agitating");
    }

    @Test
    void pullsInWhenNotStalled() {
        FakeEvilIntakeIO io = new FakeEvilIntakeIO();
        EvilIntake intake = new EvilIntake(io, 0);
        FunnelAgitate agitate = new FunnelAgitate(intake, () -> true);

        agitate.execute();
        assertEquals(EvilIntakePosition.in.getAngle(), io.lastCommandedPositionRotations, 1e-9);
    }

    @Test
    void backsOffToOutOnceStalledInsteadOfForcingItIn() throws InterruptedException {
        FakeEvilIntakeIO io = new FakeEvilIntakeIO();
        EvilIntake intake = new EvilIntake(io, 0);
        FunnelAgitate agitate = new FunnelAgitate(intake, () -> true);

        // Drive a real in-direction stall (a jammed piece) through the intake first, same
        // as if multiple pieces tried to funnel through at once.
        io.positionRotations = 8.0;
        io.statorCurrentAmps = 30.0;
        intake.evilyummy(EvilIntakePosition.in);
        long deadline = System.currentTimeMillis() + 1000;
        while (System.currentTimeMillis() < deadline && !intake.isStalled()) {
            intake.periodic();
            Thread.sleep(20);
        }
        assertTrue(intake.isStalled());

        agitate.execute();
        assertEquals(EvilIntakePosition.out.getAngle(), io.lastCommandedPositionRotations, 1e-9,
            "should back off instead of continuing to force `in` while stalled");
    }
}
