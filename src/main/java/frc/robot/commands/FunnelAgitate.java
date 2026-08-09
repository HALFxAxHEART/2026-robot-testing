package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EvilIntakePosition;
import frc.robot.subsystems.intake.EvilIntake;

/**
 * Default command for the intake: whenever nothing else is actively using it (the driver
 * isn't holding the collect button, no auto step is driving it) and {@code shouldAgitate}
 * says a shot is actually being commanded, gently cycles the rack in and out to help funnel
 * fuel into the hopper instead of just sitting still.
 *
 * <p>Deliberately does NOT try to force multiple game pieces in on one pull -- that binds
 * the mechanism. The moment {@link EvilIntake#isStalled()} reports it can't pull in any
 * further, this backs off toward `out` instead of forcing it, then tries pulling in again.
 * Because {@link EvilIntake} only auto-retracts on a stall detected while heading toward
 * `out` (a wall/robot hit), an in-direction stall here is left entirely to this command to
 * react to -- see EvilIntake.isWallHitStall().
 *
 * <p>Uses the full `in`/`out` range (the only two positions currently defined) rather than a
 * smaller "just barely open" funnel position, since a real funnel-specific setpoint would
 * need real intake geometry that isn't measured yet. If the full range turns out to be a
 * bigger swing than the funnel actually needs, that's the constant to add once you have it.
 */
public class FunnelAgitate extends Command {
    private final EvilIntake evilIntake;
    private final BooleanSupplier shouldAgitate;

    public FunnelAgitate(EvilIntake evilIntake, BooleanSupplier shouldAgitate) {
        this.evilIntake = evilIntake;
        this.shouldAgitate = shouldAgitate;
        addRequirements(evilIntake);
    }

    @Override
    public void execute() {
        if (!shouldAgitate.getAsBoolean()) {
            // Nothing to do -- hold wherever it already is instead of fighting for the
            // subsystem when there's no reason to move it.
            return;
        }

        if (evilIntake.isStalled()) {
            evilIntake.evilyummy(EvilIntakePosition.out);
        } else {
            evilIntake.evilyummy(EvilIntakePosition.in);
        }
        evilIntake.evileryummy(-1);
    }

    @Override
    public void end(boolean interrupted) {
        evilIntake.evileryummy(0);
    }

    @Override
    public boolean isFinished() {
        return false; // default command -- runs for as long as nothing else needs the intake
    }
}
