package frc.robot.subsystems.hood;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Exercises Hood.atSetpoint() through a fake IO, without any real hardware. */
class HoodTest {

    /** Fake IO that just echoes back whatever position was last commanded. */
    private static class FakeHoodIO implements HoodIO {
        double positionRotations = 0.0;

        @Override
        public void updateInputs(HoodIOInputs inputs) {
            inputs.positionRotations = positionRotations;
        }

        @Override
        public void setPosition(double positionRotations) {
            this.positionRotations = positionRotations;
        }
    }

    @Test
    void notAtSetpointWhileFarAway() {
        FakeHoodIO io = new FakeHoodIO();
        Hood hood = new Hood(io);
        hood.goTo(-1.0);
        io.positionRotations = 0.0;
        hood.periodic();
        assertFalse(hood.atSetpoint());
    }

    @Test
    void atSetpointOnceWithinTolerance() {
        FakeHoodIO io = new FakeHoodIO();
        Hood hood = new Hood(io);
        hood.goTo(-1.0);
        io.positionRotations = -1.4; // within the 0.5 tolerance
        hood.periodic();
        assertTrue(hood.atSetpoint());
    }

    @Test
    void notAtSetpointJustOutsideTolerance() {
        FakeHoodIO io = new FakeHoodIO();
        Hood hood = new Hood(io);
        hood.goTo(-1.0);
        io.positionRotations = -1.6; // just outside the 0.5 tolerance
        hood.periodic();
        assertFalse(hood.atSetpoint());
    }

    @Test
    void atSetpointAtRestWhenNeverCommanded() {
        // Setpoint defaults to 0.0 -- a hood sitting at rest (position 0) before any command
        // should already read as "at setpoint" rather than needing a command first.
        FakeHoodIO io = new FakeHoodIO();
        Hood hood = new Hood(io);
        io.positionRotations = 0.0;
        hood.periodic();
        assertTrue(hood.atSetpoint());
    }
}
