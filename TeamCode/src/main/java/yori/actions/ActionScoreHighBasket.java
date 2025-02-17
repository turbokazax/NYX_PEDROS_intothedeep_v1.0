package yori.actions;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import yori.mechas.Lift;
import yori.mechas.Outtake;

public class ActionScoreHighBasket {
    private Lift lift;
    private Outtake outtake;
    private enum SequenceState{
        MOVE_LIFT_UP,
        MOVE_OUTTAKE_DOWN,
        RELEASE_CLAW,
        MOVE_OUTTAKE_UP,
        MOVE_LIFT_DOWN,
        DISABLED
    }

    private SequenceState sequenceState = SequenceState.DISABLED;

    private ElapsedTime actionTimer;
    public ActionScoreHighBasket(Outtake outtake, Lift lift){
        this.outtake = outtake;
        this.lift = lift;
        actionTimer = new ElapsedTime();
    }
    private double actionElapsedTime = 0;
    private boolean isTimeElapsed(double time, double voltage) {
        actionElapsedTime = actionTimer.milliseconds();
        return actionElapsedTime >= time * (12.0 / voltage);
    }

    public void update(GamepadEx scorerOp, Telemetry telemetry, double voltage){
        if(scorerOp.wasJustPressed(GamepadKeys.Button.A) && sequenceState == SequenceState.DISABLED){
            actionTimer.reset();
            sequenceState = SequenceState.MOVE_LIFT_UP;
        }
        switch(sequenceState){
            case MOVE_LIFT_UP:
                if(lift.updateLiftTarget(lift.LIFT_TARGET_HB, 150) && isTimeElapsed(1, voltage)){
                    sequenceState = SequenceState.MOVE_OUTTAKE_DOWN;
                    actionTimer.reset();
                }
                break;
            case MOVE_OUTTAKE_DOWN:
//                if(isTimeElapsed(200, voltage)){
//                    sequenceState = SequenceState.RELEASE_CLAW;
//                    actionTimer.reset();
//                }
//                outtake.updateWristTarget(0);
//                outtake.updateGearTarget(0.4);
                if(scorerOp.wasJustPressed(GamepadKeys.Button.A)){
                    outtake.updateWristTarget(0);
                    outtake.updateGearTarget(0.4);
                    outtake.updateClawTarget(1);
                    if(isTimeElapsed(1, voltage)) {
                        sequenceState = SequenceState.RELEASE_CLAW;
                        actionTimer.reset();
                    }
                }
                break;
            case RELEASE_CLAW:
//                outtake.updateClawTarget(1);
                if(isTimeElapsed(500, voltage)){
                    sequenceState = SequenceState.MOVE_OUTTAKE_UP;
                    actionTimer.reset();
                }
                break;
            case MOVE_OUTTAKE_UP:
                outtake.updateGearTarget(0.5);
                outtake.updateWristTarget(0.15);
//                outtake.updateClawTarget(0.5);
                if(isTimeElapsed(200, voltage)){
                    sequenceState = SequenceState.MOVE_LIFT_DOWN;
                    actionTimer.reset();
                }
                break;
            case MOVE_LIFT_DOWN:
                if(lift.updateLiftTarget(0, 100) && isTimeElapsed(1, voltage)){
                    sequenceState = SequenceState.DISABLED;
                    actionTimer.reset();
                }
                break;

        }
        telemetry.addData("HB_SCORING_STATE", sequenceState);
    }
}
