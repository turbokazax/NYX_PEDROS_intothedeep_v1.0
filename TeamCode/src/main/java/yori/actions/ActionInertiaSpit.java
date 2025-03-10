package yori.actions;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import yori.mechas.Outtake;

public class ActionInertiaSpit {
    private Outtake outtake;
    private enum SequenceState{
        RAZ,
        DVA,
        DISABLED
    }

    private SequenceState sequenceState = SequenceState.DISABLED;
    private ElapsedTime actionTimer;


    public ActionInertiaSpit(Outtake outtake){
        this.outtake = outtake;
        actionTimer = new ElapsedTime();
    }

    private double actionElapsedTime = 0;

    private boolean isTimeElapsed(double time, double voltage) {
//        actionElapsedTime += actionTimer.milliseconds();
        actionElapsedTime = actionTimer.milliseconds();
        return actionElapsedTime >= time * (12.0 / voltage);
    }

    public void update(GamepadEx scorerOp, Telemetry telemetry, double voltage){
        if(sequenceState == SequenceState.DISABLED && scorerOp.wasJustPressed(GamepadKeys.Button.Y)){
            sequenceState = SequenceState.DVA;
            actionTimer.reset();
        }
        switch (sequenceState){
            case RAZ:
//                outtake.updateGearTarget(1-ActionTransfer.GEAR_OFFSET);
//                outtake.updateWristTarget(1);
                if(isTimeElapsed(200, voltage)){
                    sequenceState = SequenceState.DVA;
                    actionTimer.reset();
                }
                break;
            case DVA:
                outtake.updateGearTarget(0);
                outtake.updateWristTarget(1);
                if(isTimeElapsed(200, voltage)){
                    outtake.updateClawTarget(1);
                    outtake.updateWristTarget(0);
                    if(isTimeElapsed(350, voltage)){
                        outtake.updateGearTarget(0.5);
                        outtake.updateWristTarget(0.15);
                        actionTimer.reset();
                        sequenceState = SequenceState.DISABLED;
                    }
                }
                break;
        }
    }
}
