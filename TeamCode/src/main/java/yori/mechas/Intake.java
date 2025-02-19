package yori.mechas;

import static yori.teleops.PardusTeleOP.alliance;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import yori.actions.ActionPopravka;
import yori.sensors.ColorSensor;

@Config
public class Intake {
    private ServoEx leftIntakeArm;
    private ServoEx rightIntakeArm;
    private CRServo leftIntakeRoller;
    private CRServo rightIntakeRoller;
    public static double leftArmTarget;
    public static double rightArmTarget;
    public enum ArmState{
        DOWN,
        MIDDLE,
        UP
    }

    public enum RollerState{
        INTAKE,
        REJECT,
        HOLD,
        EMPTY
    }
    private ArmState armState;
    private RollerState rollerState;
    private boolean isIntakeActive;
    private boolean isIntakeReversed;
    private Telemetry telemetry;
    private ColorSensor colorSensor;

    private ElapsedTime popravlyalkaTimer;
    private double LEFTporpavDeltaTime = 0;
    private double RIGHTpopravDeltaTime = 0;
    public Intake(HardwareMap hw, Telemetry telemetry){
        this.telemetry = telemetry;
        this.leftIntakeArm = new SimpleServo(hw, "leftIntakeArm", -180, 180);
        this.rightIntakeArm = new SimpleServo(hw, "rightIntakeArm", -180, 180);
        this.leftIntakeRoller = new CRServo(hw, "leftIntakeRoller");
        this.rightIntakeRoller = new CRServo(hw, "rightIntakeRoller");
//        leftIntakeRoller.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//        rightIntakeRoller.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightIntakeArm.setInverted(true);
        leftIntakeRoller.setInverted(false);
        rightIntakeRoller.setInverted(true);
        leftArmTarget = 0;
        rightArmTarget = 0;
        armState = ArmState.UP;
        rollerState = RollerState.HOLD;
        isIntakeActive = false;
        isIntakeReversed = false;
        colorSensor = new ColorSensor(hw);
        popravlyalkaTimer = new ElapsedTime();
        actionPopravka = new ActionPopravka(leftIntakeRoller, rightIntakeRoller);
    }

    public void updateArmState(ArmState armState){
        this.armState = armState;
    }
    public double getArmPos(){
        return leftIntakeArm.getPosition();
    }
    public void updateRollerState(RollerState rollerState){
        this.rollerState = rollerState;
    }

    private double LEFT_PORPAV_TIME = 0.5;
    private double RIGHT_POPRAV_TIME = 0.5;

    public void set_poprav_times(double LEFT_PORPAV_TIME, double RIGHT_POPRAV_TIME){
        this.LEFT_PORPAV_TIME = LEFT_PORPAV_TIME;
        this.RIGHT_POPRAV_TIME = RIGHT_POPRAV_TIME;
    }

    private double LEFT_POPRAV_POWER_1 = 0.5;
    private double RIGHT_POPRAV_POWER_1 = -0.5;
    private double LEFT_POPRAV_POWER_2 = -0.5;
    private double RIGHT_POPRAV_POWER_2 = 0.5;
    private boolean NEED_PORPAVKA = true;

    public void set_poprav_powers(double LEFT_POPRAV_POWER_1, double RIGHT_POPRAV_POWER_1, double LEFT_POPRAV_POWER_2, double RIGHT_POPRAV_POWER_2){
        this.LEFT_POPRAV_POWER_1 = LEFT_POPRAV_POWER_1;
        this.RIGHT_POPRAV_POWER_1 = RIGHT_POPRAV_POWER_1;
        this.LEFT_POPRAV_POWER_2 = LEFT_POPRAV_POWER_2;
        this.RIGHT_POPRAV_POWER_2 = RIGHT_POPRAV_POWER_2;
    }

    private ActionPopravka actionPopravka;
    private double ARM_LOW_LIMIT = 0.5;
    public void setArmLowerLimit(double lowerLimit){
        this.ARM_LOW_LIMIT = lowerLimit;
    }
    public ArmState getArmState(){
        return armState;
    }
    public void updateIntake(double voltage){
        boolean correctColor = colorSensor.updateColorSensor(alliance);
        colorSensor.showTelemetry(telemetry);
        //TODO: REMOVE AFTER TESTING

        switch(armState){
            case UP:
                leftArmTarget = 1;
                rightArmTarget = 1;
                break;
            case DOWN:
                leftArmTarget = ARM_LOW_LIMIT; // A.K.A GROUND CLEARANCE of the intake in lowest pos.
                rightArmTarget = ARM_LOW_LIMIT;
                break;
            case MIDDLE:
                leftArmTarget = 0.5;
                rightArmTarget = 0.5;
                break;
        }
        telemetry.addData("ARM POS", getArmPos());
        leftIntakeArm.setPosition(leftArmTarget);
        rightIntakeArm.setPosition(rightArmTarget);

//        telemetry.addData("NEED_POPRAVKA?", NEED_PORPAVKA);

        switch(rollerState){
            case INTAKE:
                leftIntakeRoller.set(1);
                rightIntakeRoller.set(1);
                armState = ArmState.DOWN;
                if(correctColor && colorSensor.getDistance() <= 2){
                    rollerState = RollerState.HOLD;
                }else if(colorSensor.getDistance() <= 2.4){
                    rollerState = RollerState.REJECT;
                }

                break;
            case HOLD:
                armState = ArmState.UP;
                if(colorSensor.getDistance() <= 2){
                    actionPopravka.update(telemetry, voltage);
                }
                if(!correctColor && colorSensor.getDistance() <= 3.5){
                    rollerState = RollerState.REJECT;
                }
                if(colorSensor.getDistance() >= 5){
                    rollerState = RollerState.INTAKE;
                }
                break;
            case REJECT:
                leftIntakeRoller.set(-1);
                rightIntakeRoller.set(-1);
                if(colorSensor.getDistance() >= 5.0){
                    rollerState = RollerState.EMPTY;
                }

                break;
            case EMPTY:
                NEED_PORPAVKA = false;
                leftIntakeRoller.stopMotor();
                rightIntakeRoller.stopMotor();
                if (colorSensor.getDistance() <= 6) {
                    rollerState = RollerState.INTAKE;
                }
                break;
        }
        telemetry.addData("ROLLER_STATE", rollerState);
    }

    public RollerState getRollerState(){
        return rollerState;
    }

    public void setRollerState(RollerState rollerState){
        this.rollerState = rollerState;
    }
    public void setArmState(ArmState armState){
        this.armState = armState;
    }
}
