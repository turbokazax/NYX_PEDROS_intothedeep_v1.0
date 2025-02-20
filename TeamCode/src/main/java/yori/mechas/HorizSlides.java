package yori.mechas;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import yori.teleops.PardusTeleOP;

public class HorizSlides {
    private Motor horizMotor;
    private PIDController horizPID = new PIDController(0, 0, 0);

    private double hKP, hkI, hkD = 0;

    public void updateConstants(int HORIZ_SPEED, int HORIZ_MAX_SAFE_LIMIT, int HORIZ_MIN_LIMIT) {
        this.HORIZ_SPEED = HORIZ_SPEED;
        this.HORIZ_MAX_SAFE_LIMIT = HORIZ_MAX_SAFE_LIMIT;
        this.HORIZ_MIN_LIMIT = HORIZ_MIN_LIMIT;
    }

    private ElapsedTime horizTimer;

//    private enum SlideState {
//        IDLE,
//        EXTEND,
//        RETRACT
//    }

//    private SlideState slideState = SlideState.IDLE;
    private int HORIZ_SPEED = 100;
    private int HORIZ_MAX_SAFE_LIMIT = 2000;
    private int HORIZ_MIN_LIMIT = 0;
    private double horizTarget;

    private double HORIZ_ANTITENSION_POWER_ZERO = -0.11;

    public HorizSlides(HardwareMap hw) {
        this.horizMotor = new Motor(hw, "horizMotor");
        horizMotor.resetEncoder();
        hKP = PardusTeleOP.HORIZ_KP;
        hkI = PardusTeleOP.HORIZ_KI;
        hkD = PardusTeleOP.HORIZ_KD;
        horizPID.setPID(hKP, hkI, hkD);
        horizTimer = new ElapsedTime();
        horizTimer.reset();
    }
    private double horizPos;
    private boolean allowManualInput = false;

    public void setAllowManualInput(boolean allowManualInput){
        this.allowManualInput = allowManualInput;
    }

    public void setHORIZ_ANTITENSION_POWER_ZERO(double HORIZ_ANTITENSION_POWER_ZERO){
        this.HORIZ_ANTITENSION_POWER_ZERO = HORIZ_ANTITENSION_POWER_ZERO;
    }
    public void updateHoriz(GamepadEx scorerOp, Telemetry telemetry) {
        double elapsedTime = horizTimer.milliseconds() / 1000.0;
        horizTimer.reset();
        hKP = PardusTeleOP.HORIZ_KP;
        hkI = PardusTeleOP.HORIZ_KI;
        hkD = PardusTeleOP.HORIZ_KD;
        horizPID.setPID(hKP, hkI, hkD);
//        double lt = scorerOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
//        double rt = scorerOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double inpt= scorerOp.getLeftY();
//        double rt = scorerOp.getRightY();
        double input = 0;
        if(allowManualInput){
//            if (lt > 0.6) {
//                input = 1;
//            } else if (rt > 0.6) {
//                input = -1;
//            }
            input = inpt;
        }
        horizTarget += elapsedTime * input * HORIZ_SPEED;
        if (horizTarget < HORIZ_MIN_LIMIT) {
            horizTarget = HORIZ_MIN_LIMIT;
        }
        if (horizTarget > HORIZ_MAX_SAFE_LIMIT) {
            horizTarget = HORIZ_MAX_SAFE_LIMIT;
        }

        horizPos = horizMotor.getCurrentPosition();
        double power = horizPID.calculate(horizPos, horizTarget);
//        horizMotor.set(power);
        if (horizTarget == 0 && Math.abs(horizPos - 0) <= 50) {
            horizPos = 0;
//            horizTarget = -100000000;
            horizMotor.set(HORIZ_ANTITENSION_POWER_ZERO);
        } else {
            horizMotor.set(power);
        }
        telemetry.addData("HORIZ_POS:", horizPos);
        telemetry.addData("HORIZ_TARGET:", horizTarget);
        telemetry.addData("HORIZ_INPUT_MANUAL:", inpt);
//        telemetry.addData("RT:", rt);
        telemetry.addData("HORIZ_POWER:", power);
        telemetry.addData("ALL0W_MANUAL", allowManualInput);
    }

    public void updateHoriz_AUTO(Telemetry telemetry) {
        double elapsedTime = horizTimer.milliseconds() / 1000.0;
        horizTimer.reset();
        hKP = PardusTeleOP.HORIZ_KP;
        hkI = PardusTeleOP.HORIZ_KI;
        hkD = PardusTeleOP.HORIZ_KD;
        horizPID.setPID(hKP, hkI, hkD);
//        horizTarget += elapsedTime * input * HORIZ_SPEED;
        if (horizTarget < HORIZ_MIN_LIMIT) {
            horizTarget = HORIZ_MIN_LIMIT;
        }
        if (horizTarget > HORIZ_MAX_SAFE_LIMIT) {
            horizTarget = HORIZ_MAX_SAFE_LIMIT;
        }

        horizPos = horizMotor.getCurrentPosition();
        double power = horizPID.calculate(horizPos, horizTarget);
//        horizMotor.set(power);
        if (horizTarget == 0 && Math.abs(horizPos - 0) <= 100) {
            horizPos = 0;
//            horizTarget = -100000000;
            horizMotor.set(HORIZ_ANTITENSION_POWER_ZERO);
        } else {
            horizMotor.set(power);
        }
        telemetry.addData("HORIZ_POS:", horizPos);
        telemetry.addData("HORIZ_TARGET:", horizTarget);
//        telemetry.addData("LT:", lt);
//        telemetry.addData("RT:", rt);
        telemetry.addData("HORIZ_POWER:", power);
        telemetry.addData("ALL0W_MANUAL", allowManualInput);
    }

    private double tolerance = 100;

    public boolean updateHorizTarget(double horizTarget, double tolerance) {
        if(!allowManualInput){
            this.horizTarget = horizTarget;
            return horizReachedSetTarget(tolerance);
        }else{
            return false;
        }
    }

    private double getMotorPos() {
        return horizMotor.getCurrentPosition();
    }

    private boolean horizReachedSetTarget(double tolerance) {
        this.tolerance = tolerance;
        return Math.abs(horizPos - horizTarget) <= tolerance;
    }
}
