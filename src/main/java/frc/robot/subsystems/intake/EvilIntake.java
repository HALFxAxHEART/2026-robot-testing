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

     // --- STALL DETECTION (rack-and-pinion hitting a wall/robot/hard obstruction while
     // extending) -- the standard FRC pattern for this: watch for sustained high current
     // while still short of the commanded position, and treat that as "physically stuck,"
     // not "still accelerating." A firmware soft limit alone doesn't cover this case --
     // that only stops the mechanism from being commanded PAST its own programmed range, not
     // from something unexpected blocking it partway through a normal move. All three
     // numbers below are placeholders, not measured against the real intake -- kStallCurrentAmps
     // needs to sit comfortably above normal free-running current but below the 40A hard
     // limit above, and kStallDebounceSeconds needs to be long enough to ignore the normal
     // current spike from accelerating up to speed. Tune both by watching
     // EvilIntake/StatorCurrentAmps in AdvantageScope during a normal, unobstructed run.
     private static final double kStallCurrentAmps = 25.0;
     private static final double kStallDebounceSeconds = 0.25;
     private static final double kAtTargetToleranceRotations = 1.0;
     private final Debouncer stallDebouncer = new Debouncer(kStallDebounceSeconds, Debouncer.DebounceType.kRising);
     private boolean stalled = false;

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
     * so it's unit-testable without depending on real elapsed time.
     *
     * @return True if the motor is drawing stall-level current while still meaningfully short
     * of the commanded target (i.e. actually trying to get further, not just holding position).
     */
    static boolean isStallCondition(double statorCurrentAmps, double currentPositionRotations, double targetPositionRotations) {
        boolean drawingStallCurrent = statorCurrentAmps >= kStallCurrentAmps;
        boolean stillTryingToReachTarget = targetPositionRotations > currentPositionRotations + kAtTargetToleranceRotations;
        return drawingStallCurrent && stillTryingToReachTarget;
    }

    // Go-go Gadget Rotate (Makes Intake Rotate)
    public void evilyummy(EvilIntakePosition pos){ //pos is 0.36 comming in from file "Constants.java" through "RobotContainer.java" Through "EvilIntakePiece.java" to here.
        // Redirect here, not just in periodic() -- WPILib runs subsystem periodic() BEFORE
        // any command's execute() each cycle, so a command that keeps re-calling evilyummy(out)
        // every cycle (e.g. evilestyummy()) would otherwise immediately undo whatever periodic()
        // just commanded. Checking `stalled` here instead means every caller gets redirected,
        // regardless of call order or how often it's called.
        EvilIntakePosition target = (pos == EvilIntakePosition.out && stalled) ? EvilIntakePosition.in : pos;

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

        // Clear the latch once actually back home -- not the instant the stall condition
        // formula goes false (which happens the moment periodic() below redirects the target
        // to `in`, well before the mechanism has physically moved). Without this, a command
        // that keeps asking for `out` every cycle (e.g. an auto's DropIntake) would immediately
        // retry and re-stall against the same obstruction every cycle instead of actually
        // retracting first.
        boolean nearHome = Math.abs(inputs.positionRotations - EvilIntakePosition.in.getAngle()) <= kAtTargetToleranceRotations;
        if (stalled && nearHome) {
            stalled = false;
        }

        // Only worth checking once something has actually commanded a target -- otherwise
        // hitPointValue is still its 0.0 default and the check below is meaningless.
        boolean stallConditionNow = hitPoint
            && isStallCondition(inputs.statorCurrentAmps, inputs.positionRotations, hitPointValue);
        if (stallDebouncer.calculate(stallConditionNow)) {
            stalled = true;
        }

        if (stalled) {
            // Hit something on the way out -- stop fighting it and bring the intake back in.
            // evilyummy() itself also redirects out->in while stalled (see there for why that
            // matters), so this call is really just for commands like EvilIntakePiece that only
            // issue a position command once at initialize() and never again on their own.
            evilyummy(EvilIntakePosition.in);
        }
        Logger.recordOutput("EvilIntake/Stalled", stalled);

        SmartDashboard.putBoolean("Intake HitPoint", hitPoint);
        SmartDashboard.putNumber("Intake HitPointValue", hitPointValue);
        // positionRotations is already published via Logger.processInputs above -- no
        // need to duplicate it through a second NT write.
    }
}
