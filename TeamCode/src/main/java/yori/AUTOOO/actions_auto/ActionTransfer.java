package yori.AUTOOO.actions_auto;

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
    public static double GEAR_MIDDLE = 0;
    public static double GEAR_TARGET_INERTIA_1 = 0;
    public static double GEAR_TARGET_INERTIA_2 = 0;
    public static int TRANSFER_TIMER_0 = 0;
    public static int TRANSFER_TIMER_1 = 0;
    public static int TRANSFER_TIMER_2 = 0;
    public static int TRANSFER_TIMER_3 = 0;
    public static double GEAR_TARGET_TRANSFER = 0;
    public static double GEAR_TARGET_HB = 0;
    public static int INERTIA_TIMER_1 = 0;
    public static double WRIST_TARGET_TRANSFER = 0;
    public void updateConstants(double GEAR_OFFSET,double GEAR_MIDDLE,double GEAR_TARGET_INERTIA_1, double GEAR_TARGET_INERTIA_2, int TRANSFER_TIMER_0, int TRANSFER_TIMER_1, int TRANSFER_TIMER_2, int TRANSFER_TIMER_3, double GEAR_TARGET_TRANSFER, double GEAR_TARGET_HB, int INERTIA_TIMER_1, double WRIST_TRANSFER_TARGET) {
        this.GEAR_OFFSET = GEAR_OFFSET;
        this.GEAR_MIDDLE = GEAR_MIDDLE;
        this.GEAR_TARGET_INERTIA_1 = GEAR_TARGET_INERTIA_1;
        this.GEAR_TARGET_INERTIA_2 = GEAR_TARGET_INERTIA_2;
        this.TRANSFER_TIMER_0 = TRANSFER_TIMER_0;
        this.TRANSFER_TIMER_1 = TRANSFER_TIMER_1;
        this.TRANSFER_TIMER_2 = TRANSFER_TIMER_2;
        this.TRANSFER_TIMER_3 = TRANSFER_TIMER_3;
        this.GEAR_TARGET_TRANSFER = GEAR_TARGET_TRANSFER;
        this.GEAR_TARGET_HB = GEAR_TARGET_HB;
        this.INERTIA_TIMER_1 = INERTIA_TIMER_1;
        this.WRIST_TARGET_TRANSFER = WRIST_TRANSFER_TARGET;
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
                outtake.updateWristTarget(WRIST_TARGET_TRANSFER);
                outtake.updateClawTarget(1);
                if (isTimeElapsed(TRANSFER_TIMER_0, voltage)) {
                    sequenceState = SequenceState.MOVE_OUTTAKE_DOWN;
//                    actionElapsedTime = 0;
                    actionTimer.reset();
                }
                break;
            case MOVE_OUTTAKE_DOWN: // delta = 200ms?
//                outtake.updateGearTarget(1-GEAR_OFFSET);
//                outtake.updateWristTarget(0);
//                outtake.updateClawTarget(1);
                if (isTimeElapsed(1, voltage)) {
                    sequenceState = SequenceState.MOVE_INTAKE_UP_1;
//                    actionElapsedTime = 0;
                    actionTimer.reset();
                }
                break;
            case MOVE_INTAKE_UP_1: // 350?
//                actionTimer.reset();
                intake.updateArmState(Intake.ArmState.UP);
                if (isTimeElapsed(TRANSFER_TIMER_1, voltage)) {
                    actionTimer.reset();
                    sequenceState = SequenceState.CLOSE_CLAW;
                }
                break;
            case CLOSE_CLAW: //delta = 250ms;
                outtake.updateClawTarget(0.5);
//                sequenceState = SequenceState.INTAKE_RELEASE_SAMPLE;
                if (isTimeElapsed(TRANSFER_TIMER_2, voltage)) {
                    actionTimer.reset();
                    sequenceState = SequenceState.INTAKE_RELEASE_SAMPLE;
                }
                break;
            case INTAKE_RELEASE_SAMPLE: //delta = 200ms;
                intake.updateRollerState(Intake.RollerState.REJECT);
                if (isTimeElapsed(1, voltage)) {
                    actionTimer.reset();
                    sequenceState = SequenceState.MOVE_OUTTAKE_UP_SCORING;
                }
                break;
            case MOVE_OUTTAKE_UP_SCORING: //delta = 300ms?
                outtake.updateGearTarget(GEAR_MIDDLE); // remove 1, set 0.5 so arm at 90 deg.
                outtake.updateWristTarget(0.15);
                if (isTimeElapsed(TRANSFER_TIMER_3, voltage)) {
                    if(willDoInertiaSpitForSpecimen){
                        outtake.updateClawTarget(1);
                        outtake.updateGearTarget(GEAR_TARGET_INERTIA_1);
                        outtake.updateWristTarget(0);
                        if(isTimeElapsed(450, voltage)){
                            outtake.updateGearTarget(GEAR_MIDDLE);
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
