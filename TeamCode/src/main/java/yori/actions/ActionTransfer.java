package yori.actions;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import yori.mechas.HorizSlides;
import yori.mechas.Intake;
import yori.mechas.Outtake;

public class ActionTransfer {
    private Outtake outtake;
    private Intake intake;
    private HorizSlides horizSlides;

    private enum SequenceState {
        MOVE_INTAKE_DOWN_1,
        MOVE_OUTTAKE_DOWN,
        MOVE_INTAKE_UP_1,
        CLOSE_CLAW,
        INTAKE_RELEASE_SAMPLE,
        MOVE_OUTTAKE_UP_SCORING,
        DISABLED
    }

    private static SequenceState sequenceState = SequenceState.DISABLED;

    public ActionTransfer(Outtake outtake, Intake intake, HorizSlides horizSlides) {
        this.outtake = outtake;
        this.intake = intake;
        this.horizSlides = horizSlides;
        this.actionTimer = new ElapsedTime();
//        this.voltage = voltage;
    }

    private ElapsedTime actionTimer;

    //    private double voltage;
    private double getNormalizedTime(double time, double voltage) {
        return time * 12.0 / voltage;
    }

    private double actionElapsedTime = 0;

    private boolean isTimeElapsed(double time, double voltage) {
//        actionElapsedTime += actionTimer.milliseconds();
        actionElapsedTime = actionTimer.milliseconds();
        return actionElapsedTime >= time * (12.0 / voltage);
    }

    private double deltaTime = 3000;
    public static double GEAR_OFFSET = 0;

    public void updateConstants(double... consts) {
        this.GEAR_OFFSET = consts[0];
    }

    public void updateDeltaTime(double deltaTime) {
        this.deltaTime = deltaTime;
    }

    public void runOnce(Intake.ArmState armState) {
        // if disabled
        if (sequenceState == SequenceState.DISABLED) {
            actionTimer.reset();
//            sequenceState = SequenceState.MOVE_INTAKE_DOWN_1;
            if(armState != Intake.ArmState.UP){
                sequenceState = SequenceState.MOVE_INTAKE_DOWN_1;
            }else{
                sequenceState = SequenceState.MOVE_INTAKE_UP_1;
            }
        }
    }

    private boolean willDoInertiaSpitForSpecimen = false;
    public void setWillDoInertiaSpitForSpecimen(boolean willDoInertiaSpitForSpecimen){
        this.willDoInertiaSpitForSpecimen = willDoInertiaSpitForSpecimen;
    }

    public void updateTimings(double... timings){
        STEP_1_TIMING = timings[0];
        STEP_2_TIMING = timings[1];
        STEP_3_TIMING = timings[2];
        STEP_4_TIMING = timings[3];
        STEP_5_TIMING = timings[4];
        STEP_6_TIMING = timings[5];
    }

    private double STEP_1_TIMING = 500; // ms
    private double STEP_2_TIMING = 1;
    private double STEP_3_TIMING = 200;
    private double STEP_4_TIMING = 250;
    private double STEP_5_TIMING = 200;
    private double STEP_6_TIMING = 400;

    public void update(GamepadEx scorerOp, Telemetry telemetry, double voltage) {
//        telemetry.addData("Elapsed Time", actionElapsedTime);
//        if (scorerOp.wasJustPressed(GamepadKeys.Button.Y) && sequenceState == SequenceState.DISABLED) {
//        if(sequenceState == SequenceState.DISABLED){
//            actionTimer.reset();
//            sequenceState = SequenceState.MOVE_INTAKE_DOWN_1;
//        }
        switch (sequenceState) {
            case MOVE_INTAKE_DOWN_1: // delta = 200ms?
                intake.updateArmState(Intake.ArmState.UP);
                outtake.updateWristTarget(0);
                outtake.updateClawTarget(1);
                if (isTimeElapsed(STEP_1_TIMING, voltage)) {
                    sequenceState = SequenceState.MOVE_OUTTAKE_DOWN;
//                    actionElapsedTime = 0;
                    actionTimer.reset();
                }
                break;
            case MOVE_OUTTAKE_DOWN: // delta = 200ms?
//                outtake.updateGearTarget(1-GEAR_OFFSET);
//                outtake.updateWristTarget(0);
//                outtake.updateClawTarget(1);
                if (isTimeElapsed(STEP_2_TIMING, voltage)) {
                    sequenceState = SequenceState.MOVE_INTAKE_UP_1;
//                    actionElapsedTime = 0;
                    actionTimer.reset();
                }
                break;
            case MOVE_INTAKE_UP_1: // 350?
//                actionTimer.reset();
                intake.updateArmState(Intake.ArmState.UP);
                if (isTimeElapsed(STEP_3_TIMING, voltage)) {
                    actionTimer.reset();
                    sequenceState = SequenceState.CLOSE_CLAW;
                }
                break;
            case CLOSE_CLAW: //delta = 250ms;
                outtake.updateClawTarget(0.5);
//                sequenceState = SequenceState.INTAKE_RELEASE_SAMPLE;
                if (isTimeElapsed(STEP_4_TIMING, voltage)) {
                    actionTimer.reset();
                    sequenceState = SequenceState.INTAKE_RELEASE_SAMPLE;
                }
                break;
            case INTAKE_RELEASE_SAMPLE: //delta = 200ms;
                intake.updateRollerState(Intake.RollerState.REJECT);
                if (isTimeElapsed(STEP_5_TIMING, voltage)) {
                    actionTimer.reset();
                    sequenceState = SequenceState.MOVE_OUTTAKE_UP_SCORING;
                }
                break;
            case MOVE_OUTTAKE_UP_SCORING: //delta = 300ms?
                outtake.updateGearTarget(0.5); // remove 1, set 0.5 so arm at 90 deg.
                outtake.updateWristTarget(0.15);
                if (isTimeElapsed(STEP_6_TIMING, voltage)) {
                    if(willDoInertiaSpitForSpecimen){
                        outtake.updateClawTarget(1);
                        outtake.updateGearTarget(0.35);
                        outtake.updateWristTarget(0);
                        if(isTimeElapsed(650, voltage)){
                            outtake.updateGearTarget(0.5);
                            outtake.updateWristTarget(0.15);
                        }
                    }else{
                        outtake.updateClawTarget(0.5);
                    }
                    intake.setRollerState(Intake.RollerState.EMPTY);
                    actionTimer.reset();
                    sequenceState = SequenceState.DISABLED;
                }
                break;
            case DISABLED:
//                outtake.updateGearTarget(0.5);
//                    outtake.updateClawTarget(1);
//                    outtake.updateWristTarget(0.15);
                willDoInertiaSpitForSpecimen = false;
                break;
        }
        telemetry.addData("TRANSFER STAGE:", sequenceState);
    }
}
