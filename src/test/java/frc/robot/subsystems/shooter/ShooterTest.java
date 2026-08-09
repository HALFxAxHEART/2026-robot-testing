package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Exercises Shooter.isShooterReady() through a fake IO, without any real hardware. */
class ShooterTest {

    /** Fake IO that just echoes back whatever velocity was last commanded. */
    private static class FakeShooterIO implements ShooterIO {
        double velocityRPS = 0.0;

        @Override
        public void updateInputs(ShooterIOInputs inputs) {
            inputs.velocityRPS = velocityRPS;
        }

        @Override
        public void setVelocity(double velocityRPS) {
            this.velocityRPS = velocityRPS;
        }
    }

    @Test
    void notReadyBeforeAnySetpointIsCommanded() {
        Shooter shooter = new Shooter(new FakeShooterIO());
        shooter.periodic();
        assertFalse(shooter.isShooterReady(2));
    }

    @Test
    void notReadyWhileFarFromSetpoint() {
        FakeShooterIO io = new FakeShooterIO();
        Shooter shooter = new Shooter(io);
        shooter.goShoot(40.0);
        io.velocityRPS = 10.0; // hasn't spun up yet
        shooter.periodic();
        assertFalse(shooter.isShooterReady(2));
    }

    @Test
    void readyOnceWithinRangeOfSetpoint() {
        FakeShooterIO io = new FakeShooterIO();
        Shooter shooter = new Shooter(io);
        shooter.goShoot(40.0);
        io.velocityRPS = 39.0; // within the range=2 tolerance
        shooter.periodic();
        assertTrue(shooter.isShooterReady(2));
    }

    @Test
    void notReadyJustOutsideRange() {
        FakeShooterIO io = new FakeShooterIO();
        Shooter shooter = new Shooter(io);
        shooter.goShoot(40.0);
        io.velocityRPS = 37.9; // just outside the range=2 tolerance
        shooter.periodic();
        assertFalse(shooter.isShooterReady(2));
    }

    @Test
    void stopResetsSetpointSoReadyGoesFalse() {
        FakeShooterIO io = new FakeShooterIO();
        Shooter shooter = new Shooter(io);
        shooter.goShoot(40.0);
        io.velocityRPS = 40.0;
        shooter.periodic();
        assertTrue(shooter.isShooterReady(2));

        shooter.stop();
        shooter.periodic();
        assertFalse(shooter.isShooterReady(2));
    }
}
