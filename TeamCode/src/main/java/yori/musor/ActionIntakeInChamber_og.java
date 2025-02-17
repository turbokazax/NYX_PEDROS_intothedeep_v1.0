package yori.musor;

import static yori.actions.ActionTransfer.GEAR_OFFSET;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import yori.actions.ActionTransfer;
import yori.mechas.HorizSlides;
import yori.mechas.Intake;
import yori.mechas.Outtake;

public class ActionIntakeInChamber_og {
    private Intake intake;
    private HorizSlides horizSlides;
    private Outtake outtake;

    private ElapsedTime actionTimer;

    public ActionIntakeInChamber_og(Intake intake, HorizSlides horizSlides, Outtake outtake) {
        this.intake = intake;
        this.horizSlides = horizSlides;
        this.outtake = outtake;
        this.actionTimer = new ElapsedTime();
    }

    private enum SequenceState {
        MOVE_INTAKE_MID,
        HORIZ_ZERO,
        HORIZ_ZERO_ACTIVE,
        HORIZ_MID,
        HORIZ_LONG,
        SCOOPING,
        DISABLED
    }

    private SequenceState sequenceState = SequenceState.DISABLED;
    private double actionElapsedTime = 0;

    private boolean isTimeElapsed(double time, double voltage) {
//        actionElapsedTime += actionTimer.milliseconds();
        actionElapsedTime = actionTimer.milliseconds();
        return actionElapsedTime >= time * (12.0 / voltage);
    }

//    private ActionTransfer actionTransfer = new ActionTransfer(outtake, intake, horizSlides);
    SequenceState prevState = SequenceState.DISABLED;

    public void update(GamepadEx scorerOp, Telemetry telemetry, double voltage, ActionTransfer actionTransfer) {
        if (sequenceState == SequenceState.DISABLED) {
            sequenceState = SequenceState.HORIZ_ZERO;
            actionTimer.reset();
        }
        switch (sequenceState) {
//            case MOVE_INTAKE_MID:
//                if(scorerOp.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
//                    intake.setRollerState(Intake.RollerState.HOLD);
//                    intake.setArmState(Intake.ArmState.MIDDLE);
//                    if (isTimeElapsed(200, voltage)) {
//                        sequenceState = SequenceState.HORIZ_ZERO;
//                        actionTimer.reset();
//                    }
//                }
//                break;
            case HORIZ_ZERO:
                if (horizSlides.updateHorizTarget(0, 20)) {
                    if(scorerOp.wasJustPressed(GamepadKeys.Button.X)){
//                        if(intake.getRollerState() == Intake.RollerState.INTAKE && !scorerOp.wasJustReleased(GamepadKeys.Button.X)){
//                            outtake.updateClawTarget(0.5);
//                            outtake.updateWristTarget(0.15);
//                            intake.setArmState(Intake.ArmState.MIDDLE);
//                            intake.setRollerState(Intake.RollerState.HOLD);
//                        }else{
                            outtake.updateGearTarget(1-GEAR_OFFSET);
                            outtake.updateClawTarget(1);
                            outtake.updateWristTarget(0);
                            intake.setRollerState(Intake.RollerState.INTAKE);
                            intake.setArmState(Intake.ArmState.DOWN);
//                        }
                    }else{
//                        intake.setRollerState(Intake.RollerState.HOLD);
//                        intake.setArmState(Intake.ArmState.UP);
//                        outtake.updateClawTarget(1);
//                        if(prevState == SequenceState.HORIZ_LONG || prevState == SequenceState.HORIZ_MID){
//                            if(scorerOp.wasJustPressed(GamepadKeys.Button.Y)){
//                                actionTransfer.setWillDoInertiaSpitForSpecimen(true);
//                            }
                            if(intake.getRollerState() == Intake.RollerState.HOLD) {
                                scorerOp.gamepad.rumble(200);
//                                actionTransfer.runOnce();
                            }
                    }
                    if(scorerOp.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                        outtake.updateGearTarget(1-GEAR_OFFSET);
                        outtake.updateWristTarget(0);
                        outtake.updateClawTarget(1);
                        prevState = SequenceState.HORIZ_ZERO;
                        intake.setRollerState(Intake.RollerState.HOLD);
                        intake.setArmState(Intake.ArmState.MIDDLE);
                        if (isTimeElapsed(400, voltage)) {
                            sequenceState = SequenceState.HORIZ_MID;
                            actionTimer.reset();
                        }
                    }
                }
                break;
            case HORIZ_MID:
                if (horizSlides.updateHorizTarget(800, 50)) {
                    if (intake.getRollerState() == Intake.RollerState.HOLD) {
                        prevState = SequenceState.HORIZ_MID;
                        outtake.updateGearTarget(1 - GEAR_OFFSET);
                        intake.updateArmState(Intake.ArmState.MIDDLE);
                        sequenceState = SequenceState.HORIZ_ZERO;
                        actionTimer.reset();
                    }else{
                        if(scorerOp.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                            prevState = SequenceState.HORIZ_MID;
                            actionTimer.reset();
                            sequenceState = SequenceState.HORIZ_LONG;
                        }
                        if(scorerOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                            prevState = SequenceState.HORIZ_MID;
                            actionTimer.reset();
                            sequenceState = SequenceState.HORIZ_ZERO;
                        }
                    }
                }else{
                    intake.setArmState(Intake.ArmState.DOWN);
                    intake.setRollerState(Intake.RollerState.INTAKE);
                }
                break;
            case HORIZ_LONG:
                if(horizSlides.updateHorizTarget(1600, 100)){
                    if (intake.getRollerState() == Intake.RollerState.HOLD) {
                        prevState = SequenceState.HORIZ_LONG;
                        outtake.updateGearTarget(1 - GEAR_OFFSET);
                        intake.updateArmState(Intake.ArmState.MIDDLE);
                        sequenceState = SequenceState.HORIZ_ZERO;
                        actionTimer.reset();
                    }else{
                        if (scorerOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                            prevState = SequenceState.HORIZ_LONG;
                            actionTimer.reset();
                            sequenceState= SequenceState.HORIZ_MID;
                        }
                    }
                }
                break;
        }
        telemetry.addData("IN-CHAMBER_STATE", sequenceState);
    }

}
