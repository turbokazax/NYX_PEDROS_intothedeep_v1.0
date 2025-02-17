package yori.AUTOOO.actions_auto;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionPopravka {
    private CRServo leftIntakeRoller;
    private CRServo rightIntakeRoller;
    private ElapsedTime actionTimer;

    public ActionPopravka(CRServo leftIntakeRoller, CRServo rightIntakeRoller){
        this.leftIntakeRoller = leftIntakeRoller;
        this.rightIntakeRoller = rightIntakeRoller;
        actionTimer = new ElapsedTime();
    }

    private enum SequenceState{
        ROLL_IN,
        DISABLED
    }

    private SequenceState sequenceState = SequenceState.DISABLED;

    private double actionElapsedTime = 0;
    private boolean isTimeElapsed(double time, double voltage) {
//        actionElapsedTime += actionTimer.milliseconds();
        actionElapsedTime = actionTimer.milliseconds();
        return actionElapsedTime >= time * (12.0 / voltage);
    }

    public void update(Telemetry telemetry, double voltage){
        if(sequenceState == SequenceState.DISABLED){
            sequenceState = SequenceState.ROLL_IN;
            actionTimer.reset();
        }
        switch(sequenceState){
            case ROLL_IN:
                leftIntakeRoller.set(1);
                rightIntakeRoller.set(1);
                if(isTimeElapsed(300, voltage)){
                    sequenceState = SequenceState.DISABLED;
                }
                break;
            case DISABLED:
                leftIntakeRoller.stopMotor();
                rightIntakeRoller.stopMotor();
                if(isTimeElapsed(150, voltage)){
                    sequenceState=SequenceState.ROLL_IN;
                }
                break;
        }
        telemetry.addData("PORPAVKA_STATE", sequenceState);
    }
}
