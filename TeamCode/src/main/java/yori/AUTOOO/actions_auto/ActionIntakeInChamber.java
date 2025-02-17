package yori.AUTOOO.actions_auto;

import static yori.actions.ActionTransfer.GEAR_OFFSET;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import yori.mechas.HorizSlides;
import yori.mechas.Intake;
import yori.mechas.Outtake;

public class ActionIntakeInChamber {
    private Intake intake;
    private HorizSlides horizSlides;
    private Outtake outtake;

    private ElapsedTime actionTimer;

    public ActionIntakeInChamber(Intake intake, HorizSlides horizSlides, Outtake outtake) {
        this.intake = intake;
        this.horizSlides = horizSlides;
        this.outtake = outtake;
        this.actionTimer = new ElapsedTime();
    }

    public enum SequenceState {
        MOVE_INTAKE_MID,
        HORIZ_ZERO,
        HORIZ_ZERO_ACTIVE,
        HORIZ_MID,
        HORIZ_LONG,
        HORIZ_FREE,
        SCOOPING,
        DISABLED
    }

    private SequenceState sequenceState = SequenceState.DISABLED;
    private double actionElapsedTime = 0;

    private int FIELD_SAMPLE_HORIZ_TARGET = 1500;
    private boolean isTimeElapsed(double time, double voltage) {
//        actionElapsedTime += actionTimer.milliseconds();
        actionElapsedTime = actionTimer.milliseconds();
        return actionElapsedTime >= time * (12.0 / voltage);
    }

    //    private ActionTransfer actionTransfer = new ActionTransfer(outtake, intake, horizSlides);
    SequenceState prevState = SequenceState.DISABLED;

    public void setSequenceState(SequenceState sequenceState) {
        this.sequenceState = sequenceState;
    }
    public void update(Telemetry telemetry, double voltage, ActionTransfer actionTransfer) {
        if (sequenceState == SequenceState.DISABLED) {
            sequenceState = SequenceState.HORIZ_ZERO;
            actionTimer.reset();
        }

        switch (sequenceState) {
            case HORIZ_ZERO:
                horizSlides.setAllowManualInput(false);
                if (horizSlides.updateHorizTarget(0, 20)) {
                    if (intake.getRollerState() == Intake.RollerState.HOLD) {
                        actionTransfer.runOnce(intake.getArmState());
                    }
                }
                break;
            case HORIZ_MID:
                if (intake.getRollerState() == Intake.RollerState.HOLD) {
                    prevState = SequenceState.HORIZ_MID;
                    outtake.updateGearTarget(1 - GEAR_OFFSET);
                    intake.updateArmState(Intake.ArmState.MIDDLE);
                    sequenceState = SequenceState.HORIZ_ZERO;
                    actionTimer.reset();
                }
                intake.setArmState(Intake.ArmState.DOWN);
                intake.setRollerState(Intake.RollerState.INTAKE);
                horizSlides.updateHorizTarget(FIELD_SAMPLE_HORIZ_TARGET, 50);
                break;
        }
        telemetry.addData("AUTO_INTAKE_STATE", sequenceState);
    }

}
