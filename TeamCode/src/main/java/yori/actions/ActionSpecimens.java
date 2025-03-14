package yori.actions;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import yori.mechas.Lift;
import yori.mechas.Outtake;

public class ActionSpecimens {
    private Outtake outtake;
    private Lift lift;

    private enum SequenceState {
        PICK_FROM_HUMAN,
        MOVE_LIFT_UP,
        MOVE_OUTTAKE_DOWN,
        RELEASE_CLAW,
        MOVE_OUTTAKE_UP,
        MOVE_LIFT_DOWN,
        DISABLED
    }

    private SequenceState sequenceState = SequenceState.DISABLED;
    private ElapsedTime actionTimer;

    public ActionSpecimens(Outtake outtake, Lift lift) {
        this.outtake = outtake;
        this.lift = lift;
        this.actionTimer = new ElapsedTime();
    }

    private double actionElapsedTime = 0;

    private boolean isTimeElapsed(double time, double voltage) {
        actionElapsedTime = actionTimer.milliseconds();
        return actionElapsedTime >= time * (12.0 / voltage);
    }

    public SequenceState getCurrentState() {
        return sequenceState;
    }

    private double SPECI_GEAR_OFFEST = 0;

    public void setSPECI_GEAR_OFFEST(double SPECI_GEAR_OFFEST){
        this.SPECI_GEAR_OFFEST = SPECI_GEAR_OFFEST;
    }

    private int POSITION_SCORE_1;
    private int POSITION_SCORE_2;
//    private double CLAW_RELEASE_TIMING_MS = 200; // ms

    private double WRIST_OFFSET = 0;

    public void setConstants(int POSITION_SCORE_1, int POSITION_SCORE_2, double WRIST_OFFSET){
        this.POSITION_SCORE_1 = POSITION_SCORE_1;
        this.POSITION_SCORE_2 = POSITION_SCORE_2;
//        this.CLAW_RELEASE_TIMING_MS = CLAW_RELEASE_TIMING_MS;
        this.WRIST_OFFSET = WRIST_OFFSET;
    }

    public void update(GamepadEx scorerOp, Telemetry telemetry, double voltage) {
        if (scorerOp.wasJustPressed(GamepadKeys.Button.B) && sequenceState == SequenceState.DISABLED) {
            actionTimer.reset();
            sequenceState = SequenceState.PICK_FROM_HUMAN;
        }
        switch (sequenceState) {
            case PICK_FROM_HUMAN:
                outtake.updateGearTarget(0);
                outtake.updateWristTarget(0.15);
                outtake.updateClawTarget(1);
                if (scorerOp.wasJustPressed(GamepadKeys.Button.B) && isTimeElapsed(300, voltage)) {
                    outtake.updateClawTarget(0.518);
                    if (isTimeElapsed(150, voltage)) {
                        actionTimer.reset();
                        sequenceState = SequenceState.MOVE_LIFT_UP;
                    }
                }
                break;
            case MOVE_LIFT_UP:
                outtake.updateGearTarget(1-SPECI_GEAR_OFFEST);
                outtake.updateWristTarget(WRIST_OFFSET);
                if (lift.updateLiftTarget(POSITION_SCORE_1, 60)) {
//                    sequenceState = SequenceState.MOVE_OUTTAKE_DOWN;
//                    outtake.updateGearTarget(0);
//                    outtake.updateWristTarget(0);
                    if(scorerOp.wasJustPressed(GamepadKeys.Button.B) && isTimeElapsed(10, voltage)){
                        actionTimer.reset();
                        sequenceState = SequenceState.MOVE_OUTTAKE_UP;
                    }
                }
                break;
//            case MOVE_OUTTAKE_DOWN:
//                if (lift.updateLiftTarget(2000, 50) && isTimeElapsed(1, voltage)) {
//                    sequenceState = SequenceState.MOVE_OUTTAKE_UP;
//                    actionTimer.reset();
//                }
//                break;
//            case MOVE_OUTTAKE_DOWN:
//                outtake.updateGearTarget(0);
//                outtake.updateWristTarget(0);
//                if (isTimeElapsed(200, voltage)) {
//                    sequenceState = SequenceState.RELEASE_CLAW;
//                    actionTimer.reset();
//                }
//                break;
//            case RELEASE_CLAW:
//                outtake.updateClawTarget(1);
//                if (isTimeElapsed(200, voltage)) {
//                    sequenceState = SequenceState.MOVE_OUTTAKE_UP;
//                    actionTimer.reset();
//                }
//                break;
            case MOVE_OUTTAKE_UP:

//                outtake.updateWristTarget(0.15);
////                outtake.updateGearTarget(0.5);
//                if (isTimeElapsed(CLAW_RELEASE_TIMING_MS, voltage)) {
//
//                }
//                if (isTimeElapsed(400, voltage)) {
                if(lift.liftReachedCertainTarget(1900, 100)){
//                    outtake.updateWristTarget(0);
                }
                if(lift.updateLiftTarget(POSITION_SCORE_2, 100)){
                    outtake.updateGearTarget(0.5);
                    outtake.updateClawTarget(1);
                    sequenceState = SequenceState.MOVE_LIFT_DOWN;
                    actionTimer.reset();
                }
                break;
            case MOVE_LIFT_DOWN:
                if (lift.updateLiftTarget(0, 50) && isTimeElapsed(1, voltage)) {
                    outtake.updateWristTarget(0.15);
                    sequenceState = SequenceState.DISABLED;
                    actionTimer.reset();
                }
                break;
        }
        telemetry.addData("SPECIMENS_ACTION_STATE", sequenceState);
    }

}
