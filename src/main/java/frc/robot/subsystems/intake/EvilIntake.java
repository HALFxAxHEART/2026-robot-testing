package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.EvilIntakePosition;

public class EvilIntake extends SubsystemBase {
     // Same margins EvilIntakeIOTalonFX configures as firmware soft limits -- kept here as
     // the single source of truth so EvilIntakeIOSim can obey the same limits as the real
     // robot instead of duplicating (and risking drifting from) these numbers.
     public static final double kForwardSoftLimitRotations = EvilIntakePosition.out.getAngle() + 0.5;
     public static final double kReverseSoftLimitRotations = EvilIntakePosition.in.getAngle() - 0.1;

     // --- STALL DETECTION (bidirectional: hitting a wall/robot while extending, or a jammed
     // game piece while retracting) -- the standard FRC pattern for this: watch for sustained
     // high current while still short of the commanded position, and treat that as
     // "physically stuck," not "still accelerating." A firmware soft limit alone doesn't
     // cover this case -- that only stops the mechanism from being commanded PAST its own
     // programmed range, not from something unexpected blocking it partway through a normal
     // move. All three numbers below are placeholders, not measured against the real intake --
     // kStallCurrentAmps needs to sit comfortably above normal free-running current but below
     // the 40A hard limit above, and kStallDebounceSeconds needs to be long enough to ignore
     // the normal current spike from accelerating up to speed. Tune both by watching
     // EvilIntake/StatorCurrentAmps in AdvantageScope during a normal, unobstructed run.
     private static final double kStallCurrentAmps = 25.0;
     private static final double kStallDebounceSeconds = 0.25;
     private static final double kAtTargetToleranceRotations = 1.0;
     private final Debouncer stallDebouncer = new Debouncer(kStallDebounceSeconds, Debouncer.DebounceType.kRising);
     private boolean stalled = false;
     // What hitPointValue was AT THE MOMENT the current stall was first detected -- frozen on
     // the rising edge, not re-read live, so the "hit a wall while extending" auto-retract
     // below can tell that apart from "jammed on a game piece while retracting" (which
     // FunnelAgitate handles itself -- see there) even after something has since changed
     // what's currently commanded.
     private double stalledTowardTarget = Double.NaN;

     // Roller speed/direction while the rotation mechanism is actively in transit (either
     // direction) -- keeps a game piece from jamming in the mechanism as the arm swings
     // through it. Same "-1 = intake" convention already used elsewhere (EvilIntakePiece).
     private static final double kTransitSpinPercent = -1.0;

     private final EvilIntakeIO io;
     private final EvilIntakeIOInputsAutoLogged inputs = new EvilIntakeIOInputsAutoLogged();

     //debugging
     boolean hitPoint = false;
     double hitPointValue;
     int intakevalueid = 0;

    /** Constructs an Intake
     * @param io The hardware IO implementation
     * @param intakeID Kept only for the "Intake ID" dashboard label
     */
    public EvilIntake(EvilIntakeIO io, int intakeID){
        this.io = io;
        intakevalueid = intakeID;
        // Constant for the life of the robot -- publish once here instead of every
        // periodic() cycle.
        SmartDashboard.putNumber("Intake ID", intakevalueid);
    }

    /**
     * @return True if the rotation motor is drawing sustained high current while still short
     * of its commanded position -- i.e. physically stuck against something, not just
     * accelerating normally. See the stall-detection constants above for the exact criteria.
     */
    public boolean isStalled() {
        return stalled;
    }

    /**
     * Pure stall-condition check, split out from the Debouncer-wrapped version in periodic()
     * so it's unit-testable without depending on real elapsed time. Direction-agnostic --
     * works the same whether currently trying to extend or retract.
     *
     * @return True if the motor is drawing stall-level current while still meaningfully far
     * from the commanded target (i.e. actually trying to get there, not just holding position
     * against a soft limit, which also draws current but isn't a stall).
     */
    static boolean isStallCondition(double statorCurrentAmps, double currentPositionRotations, double targetPositionRotations) {
        boolean drawingStallCurrent = statorCurrentAmps >= kStallCurrentAmps;
        boolean stillTryingToReachTarget =
            Math.abs(targetPositionRotations - currentPositionRotations) > kAtTargetToleranceRotations;
        return drawingStallCurrent && stillTryingToReachTarget;
    }

    /** True only when the current stall happened while trying to reach `out` (a wall/robot
     * hit while extending) -- as opposed to while trying to reach `in` (a jammed game piece
     * while retracting, which FunnelAgitate is responsible for backing off from itself). */
    private boolean isWallHitStall() {
        return stalled && stalledTowardTarget == EvilIntakePosition.out.getAngle();
    }

    // Go-go Gadget Rotate (Makes Intake Rotate)
    public void evilyummy(EvilIntakePosition pos){ //pos is 0.36 comming in from file "Constants.java" through "RobotContainer.java" Through "EvilIntakePiece.java" to here.
        // Redirect here, not just in periodic() -- WPILib runs subsystem periodic() BEFORE
        // any command's execute() each cycle, so a command that keeps re-calling evilyummy(out)
        // every cycle (e.g. evilestyummy()) would otherwise immediately undo whatever periodic()
        // just commanded. Checking here instead means every caller gets redirected, regardless
        // of call order or how often it's called. Only applies to the wall-hit case -- a
        // command deliberately asking for `out` while jammed on a piece (FunnelAgitate backing
        // off) must NOT be redirected back to `in`, or it could never back off at all.
        EvilIntakePosition target = (pos == EvilIntakePosition.out && isWallHitStall()) ? EvilIntakePosition.in : pos;

        hitPoint = true;
        hitPointValue = target.getAngle();

        io.setRotationPosition(target.getAngle());
    }

    public void evileryummy(double speed){
        io.setSpinPercent(speed);
    }

    public Command evilestyummy(EvilIntakePosition pos){
        return run(() -> evilyummy(pos));
    }

    public double getAngle(){
        return inputs.rotorPositionRotations;
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EvilIntake", inputs);

        // Clear the latch once actually at the (possibly redirected) target -- not the instant
        // the stall condition formula goes false, which can happen well before the mechanism
        // has physically moved anywhere (e.g. the moment evilyummy() redirects the target).
        // Without this, a command that keeps asking for the same thing every cycle (e.g. an
        // auto's DropIntake, or FunnelAgitate) would immediately retry and re-stall against
        // the same obstruction instead of actually clearing it first.
        boolean nearTarget = Math.abs(inputs.positionRotations - hitPointValue) <= kAtTargetToleranceRotations;
        if (stalled && nearTarget) {
            stalled = false;
        }

        // Only worth checking once something has actually commanded a target -- otherwise
        // hitPointValue is still its 0.0 default and the check below is meaningless.
        boolean stallConditionNow = hitPoint
            && isStallCondition(inputs.statorCurrentAmps, inputs.positionRotations, hitPointValue);
        if (stallDebouncer.calculate(stallConditionNow)) {
            if (!stalled) {
                // Freeze what we were headed for right as the stall starts, not on every
                // cycle it stays true -- see stalledTowardTarget's declaration for why.
                stalledTowardTarget = hitPointValue;
            }
            stalled = true;
        }

        if (isWallHitStall()) {
            // Hit something on the way out -- stop fighting it and bring the intake back in.
            // evilyummy() itself also redirects out->in for this same case (see there for why
            // that matters), so this call is really just for commands like EvilIntakePiece
            // that only issue a position command once at initialize() and never again on
            // their own. Deliberately does NOT fire for an in-direction (jammed-piece) stall --
            // FunnelAgitate is responsible for deciding what to do about that itself.
            evilyummy(EvilIntakePosition.in);
        }
        Logger.recordOutput("EvilIntake/Stalled", stalled);

        // Rollers must be spinning any time the rotation mechanism is actually moving --
        // idle rollers while the arm swings through a game piece is how it jams. This
        // applies no matter which command is driving the motion (button, auto, FunnelAgitate),
        // since it's keyed only on "commanded somewhere it hasn't reached yet." Whatever
        // happens once actually AT the target (e.g. continuing to intake while deployed) is
        // still each command's own job -- this only covers the transit itself.
        boolean inTransit = hitPoint && !nearTarget;
        if (inTransit) {
            io.setSpinPercent(kTransitSpinPercent);
        }

        // Logger.recordOutput (not SmartDashboard) so these show up in replay too, same as
        // EvilIntake/Stalled above -- positionRotations itself is already published via
        // Logger.processInputs, no need to duplicate it through a second write.
        Logger.recordOutput("EvilIntake/HitPoint", hitPoint);
        Logger.recordOutput("EvilIntake/HitPointValue", hitPointValue);
    }
}
