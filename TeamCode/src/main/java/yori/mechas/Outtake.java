package yori.mechas;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    private ServoEx gearServoLeft;
    private ServoEx gearServoRight;
    private ServoEx clawServo;
    private ServoEx wristServo;

    public Outtake(HardwareMap hw) {
        this.gearServoLeft = new SimpleServo(hw, "leftGearServo", -180, 180);
        this.gearServoRight = new SimpleServo(hw, "rightGearServo", -180, 180);
        this.gearServoRight.setInverted(false);
        this.gearServoLeft.setInverted(true);
        this.clawServo = new SimpleServo(hw, "clawServo", -180, 180);
        this.wristServo = new SimpleServo(hw, "wristServo", -180, 180);

        gearTarget = 0.5;
        clawTarget = 1; //0.5 - closed, 1 - open
        wristTarget = 0; // 0 - up, 1- down, 0.5 - mid
    }
    private double clawTarget;
    private double wristTarget;
    private double gearTarget;

//    public void updateTargets(double gearLtarget, double gearRtarget, double scoopTarget, double wristTarget ){
//        this.gearLtarget = gearLtarget;
//        this.gearRtarget = gearRtarget;
//        this.clawTarget = scoopTarget;
//        this.wristTarget = wristTarget;
//    }
    public void updateTargets(double gearTarget, double clawTarget, double wristTarget){
        this.wristTarget = wristTarget;
        this.clawTarget = clawTarget;
        this.gearTarget = gearTarget;
    }

    public void updateGearTarget(double gearTarget){
        this.gearTarget = gearTarget;
    }
    public double getGearPos(){
        return gearServoLeft.getPosition();
    }

    public void updateClawTarget(double clawTarget){
        this.clawTarget = clawTarget;
    }
    public void updateWristTarget(double wristTarget){
        this.wristTarget = wristTarget;
    }
    public void updateOuttake(){
        gearServoLeft.setPosition(gearTarget);
//        gearServoRight.setPosition(gearRtarget);
        gearServoRight.setPosition(gearTarget);
        clawServo.setPosition(clawTarget);
        wristServo.setPosition(wristTarget);
    }
}
