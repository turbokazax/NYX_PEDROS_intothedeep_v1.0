package yori.mechas;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import yori.sensors.TouchSensorGroup;
import yori.teleops.PardusTeleOP;

public class Lift {

    private double KP, KI, KD = 0.0;
    private Motor liftLmotor;
    private Motor liftRmotor;

    private double liftTarget;
    //    private double currentPos;
    public PIDController pidLeft = new PIDController(0, 0, 0);
    public PIDController pidRight = new PIDController(0, 0, 0);

    private TouchSensorGroup touchSensorGroup;
    private enum LiftState {
        IDLE_OR_CONTROLLED,
        EXTEND,
        RETRACT,
        HANGING,
    }

    private LiftState liftState = LiftState.IDLE_OR_CONTROLLED;
    private ElapsedTime liftTimer;
    private double ff = 0;
    public Lift(HardwareMap hwmap) {
        this.liftLmotor = new Motor(hwmap, "liftLmotor");
        this.liftRmotor = new Motor(hwmap, "liftRmotor");
        this.touchSensorGroup = new TouchSensorGroup(hwmap);
        liftLmotor.resetEncoder();
        liftRmotor.resetEncoder();
        liftRmotor.setInverted(true);
        double kp = PardusTeleOP.LIFT_KP;
        double ki = PardusTeleOP.LIFT_KI;
        double kd = PardusTeleOP.LIFT_KD;
        ff = PardusTeleOP.LIFT_FF;
        this.setPIDFFCoefficients(kp, ki, kd, ff);
        liftTimer = new ElapsedTime();
        liftTimer.reset();
    }

    public void setPIDFFCoefficients(double... coefs) {
        pidLeft.setPID(coefs[0], coefs[1], coefs[2]);
        pidRight.setPID(coefs[0], coefs[1], coefs[2]);
        ff = coefs[3];
    }

    private double calculateFFpower(double pos, double target){
        if(pos > 2200 && liftTarget > 2200 && target >= pos){
            double error = target - pos;
            return ff*error;
        }else{
            return 0;
        }
    }

    private boolean maxLengthReached = false;
    private boolean minLengthReached = false;

    private int LIFT_SPEED = 300;
    private int LIFT_MAX_POS_SAFE = 3500;
    private int LIFT_MIN_POS = 0;
    private double liftPower = 0;
    public void updateConstants(int LIFT_SPEED, int LIFT_MAX_POS_SAFE, int LIFT_MIN_POS, int LIFT_TARGET_HB){
        this.LIFT_SPEED = LIFT_SPEED;
        this.LIFT_MAX_POS_SAFE = LIFT_MAX_POS_SAFE;
        this.LIFT_MIN_POS = LIFT_MIN_POS;
        this.LIFT_TARGET_HB = LIFT_TARGET_HB;
    }
    public void updateLift(GamepadEx scorerOp, Telemetry telemetry) {
        double elapsedTime = liftTimer.milliseconds() / 1000.0;
        liftTimer.reset();
//        if (scorerOp.isDown(GamepadKeys.Button.LEFT_BUMPER) && !liftState.equals(LiftState.HANGING)) {
//            if (scorerOp.isDown(GamepadKeys.Button.RIGHT_BUMPER) || maxLengthReached) {
//                liftState = LiftState.IDLE_OR_CONTROLLED;
//            } else {
//                liftState = LiftState.EXTEND;
//            }
//        }
//        if (scorerOp.isDown(GamepadKeys.Button.RIGHT_BUMPER) && !liftState.equals(LiftState.HANGING)) {
//            if (scorerOp.isDown(GamepadKeys.Button.LEFT_BUMPER) || minLengthReached) {
//                liftState = LiftState.IDLE_OR_CONTROLLED;
//            } else {
//                liftState = LiftState.RETRACT;
//            }
//        }
//
//        if (scorerOp.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER) || scorerOp.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)
//                && !liftState.equals(LiftState.HANGING)) {
//            liftState = LiftState.IDLE_OR_CONTROLLED;
//        }

        switch (liftState) {
            case EXTEND:
                liftPower = 1;
                break;
            case RETRACT:
                liftPower = -1;
                break;
            case IDLE_OR_CONTROLLED:
                liftPower = 0;
                break;
            case HANGING:
                //TODO: this
                break;
        }

        liftTarget += liftPower * LIFT_SPEED * elapsedTime;

        double liftLpos = liftLmotor.getCurrentPosition();
        double liftRpos = liftRmotor.getCurrentPosition();

        if(touchSensorGroup.isLiftAtMinPos()){
            liftLpos = 0;
            liftRpos = 0;
            minLengthReached = false;
        }

        if (!liftState.equals(LiftState.HANGING)) {
            if (liftTarget >= LIFT_MAX_POS_SAFE) {
                liftTarget = LIFT_MAX_POS_SAFE;
                liftState = LiftState.IDLE_OR_CONTROLLED;
                maxLengthReached = true;
            } else {
                maxLengthReached = false;
            }
            if (liftTarget < LIFT_MIN_POS) {
                liftTarget = LIFT_MIN_POS;
//                liftLmotor.resetEncoder();
//                liftRmotor.resetEncoder();
                liftState = LiftState.IDLE_OR_CONTROLLED;
                minLengthReached = true;
            } else {
                minLengthReached = false;
            }
        }

//        if(scorerOp.wasJustPressed(GamepadKeys.Button.START)){
////            liftLmotor.resetEncoder();
////            liftRmotor.resetEncoder();
//            liftTarget = 0;
//        }
        double liftLpower = (liftState != LiftState.HANGING) ? pidLeft.calculate(liftLpos, liftTarget) : -1;
        double liftRpower = (liftState != LiftState.HANGING) ? pidRight.calculate(liftRpos, liftTarget) : -1;
//        double ffL = calculateFFpower(liftLpos, liftTarget);
        double ffL = 0;
//        double ffR = calculateFFpower(liftRpos, liftTarget);
        double ffR = 0;
//        liftLmotor.set(liftLpower+ffL);
//        liftLmotor.set(liftRpower+ffR);
//        liftRmotor.set(liftRpower+ffR);
//        if(liftReachedSetTarget(tolerance)){
//            liftLmotor.set(0);
//            liftRmotor.set(0);
//        }else{
//            liftLmotor.set(liftLpower+ffL);
//            liftRmotor.set(liftLpower+ffL);
//        }
        liftLmotor.set(liftLpower+ffL);
        liftRmotor.set(liftRpower+ffR);
        telemetry.addData("liftLpos:", liftLpos);
        telemetry.addData("liftRpos:", liftRpos);
        telemetry.addData("liftLPower:", liftLpower);
        telemetry.addData("liftRPower:", liftRpower);
        telemetry.addData("liftTarget:", liftTarget);
        telemetry.addData("liftState:", liftState);
        touchSensorGroup.addTelemetry(telemetry);
//        telemetry.addData("FF_LEFT_POWER", ffL);
//        telemetry.addData("FF_RIGHT_POWER", ffR);
    }

    public double getNormalizedMotorPos(){
//        return (liftLmotor.getCurrentPosition() + liftRmotor.getCurrentPosition()) / 2.0;
//        if(liftTarget > Math.max(liftLmotor.getCurrentPosition(), liftRmotor.getCurrentPosition())){
//            return Math.max(liftLmotor.getCurrentPosition(), liftRmotor.getCurrentPosition());
//        }else{
//            return Math.min(liftLmotor.getCurrentPosition(), liftRmotor.getCurrentPosition());
//        }
        return liftLmotor.getCurrentPosition();
    }
    public  double LIFT_TARGET_HB = 3150;
    public boolean updateLiftTarget(double target, double tolerance){
        liftTarget = target;
//        liftState = LiftState.IDLE_OR_CONTROLLED;
        double leftPos = liftLmotor.getCurrentPosition();
        double rightPos = liftRmotor.getCurrentPosition();

        return liftReachedSetTarget(tolerance);
    }
    public boolean liftReachedCertainTarget(double target, double tolerance){
        return Math.abs(getNormalizedMotorPos() - target) <= tolerance;
    }
    private double tolerance = 50;
    private boolean liftReachedSetTarget(double tolerance){
        this.tolerance = tolerance;
        return Math.abs(getNormalizedMotorPos() - liftTarget) <= tolerance;
    }

    public void prostoGiveTelemetry(Telemetry telemetry){
        double liftLpos = liftLmotor.getCurrentPosition();
        double liftRpos = liftRmotor.getCurrentPosition();
        if(touchSensorGroup.isLiftAtMinPos()) {
            liftLpos = 0;
            liftRpos = 0;
//            minLengthReached = false;
        }
        telemetry.addData("LiftLEFT_POS", liftLpos);
        telemetry.addData("LiftRIGHT_POS", liftRpos);

    }
}