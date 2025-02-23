package yori.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import yori.actions.ActionInertiaSpit;
import yori.actions.ActionIntakeInChamber;
import yori.actions.ActionScoreHighBasket;
import yori.actions.ActionSpecimens;
import yori.actions.ActionTransfer;
import yori.mechas.HorizSlides;
import yori.mechas.Intake;
import yori.mechas.Lift;
import yori.mechas.Outtake;
import yori.sensors.ColorSensor;
import yori.utils.Alliance;
import yori.utils.SpeedController;


@TeleOp(name = "PardusTeleOP")
@Config
public class PardusTeleOP extends OpMode {
    // GAMEPADS:
    public GamepadEx driverOp;
    public GamepadEx scorerOp;

    public GamepadEx giveDriver() {
        return driverOp;
    }

    // DRIVE MOTORS:
    private Motor FLMotor;
    private Motor BLMotor;
    private Motor BRMotor;
    private Motor FRMotor;

    private MotorGroup driveMotors;
    private SpeedController speedController = new SpeedController();

    // HEADING CONTROL:
    private ElapsedTime headingTimer;
    private boolean useFieldCentric = true;
    private BNO055IMUNew imu;
    private static double headingTarget = 0;
    public static double HEADING_TURN_SPEED = 180;
    public double integralSum = 0;
    public double lastError = 0;
    public static double HEADING_KP = 0.012;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.0;

    // SYSTEM:
    public static boolean SYSTEM_IN_DEBUG_MODE = false;

    public enum SystemSpeedMode {
        TURBO,
        DEFAULT,
        SLOW,
        ULTRASLOW
    }

    // MECHAS:
    private MecanumDrive drive;
    private Lift lift;
    private Outtake outtake;
    private HorizSlides horizSlides;
    private Intake intake;
    private ColorSensor colorSensor;
    public SystemSpeedMode speedMode = SystemSpeedMode.DEFAULT;


    public static Alliance alliance;


    //actions:
    private ActionTransfer actionTransfer;
    private ActionSpecimens actionSpecimens;
    private ActionScoreHighBasket actionScoreHighBasket;
    private ActionIntakeInChamber actionIntakeInChamber;
    private ActionInertiaSpit actionInertiaSpit;
//    public Gamepad getDriverOp(){
//        return gamepad1;
//    }
//    public Gamepad getScorerOp(){
//        return gamepad2;
//    }
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverOp = new GamepadEx(gamepad1);
        scorerOp = new GamepadEx(gamepad2);

        headingTimer = new ElapsedTime();
        // drive motors:
        FLMotor = new Motor(hardwareMap, "FLmotor");
        FRMotor = new Motor(hardwareMap, "FRmotor");
        BLMotor = new Motor(hardwareMap, "BLmotor");
        BRMotor = new Motor(hardwareMap, "BRmotor");

        FLMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FLMotor.setRunMode(Motor.RunMode.VelocityControl);
        FRMotor.setRunMode(Motor.RunMode.VelocityControl);
        BLMotor.setRunMode(Motor.RunMode.VelocityControl);
        BRMotor.setRunMode(Motor.RunMode.VelocityControl);
        driveMotors = new MotorGroup(FLMotor, BLMotor, BRMotor, FRMotor);
//        driveMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        driveMotors.setRunMode(Motor.RunMode.VelocityControl);
//        drive = new MecanumDrive(FLMotor, FRMotor, BLMotor, BRMotor);
        FLMotor.setInverted(true);
        BLMotor.setInverted(true);
        FRMotor.setInverted(true);
        BRMotor.setInverted(true);
        drive = new MecanumDrive(FLMotor, FRMotor, BLMotor, BRMotor);
//        speedController = new SpeedController();

        imu = hardwareMap.get(BNO055IMUNew.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN)
        );
        imu.initialize(imuParams);
        imu.resetYaw();
        lift = new Lift(hardwareMap);
        outtake = new Outtake(hardwareMap);
        horizSlides = new HorizSlides(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);
        colorSensor = new ColorSensor(hardwareMap);
        //actions:
        actionTransfer = new ActionTransfer(outtake, intake, horizSlides);
        actionSpecimens = new ActionSpecimens(outtake, lift);
        actionScoreHighBasket = new ActionScoreHighBasket(outtake, lift);
        actionIntakeInChamber = new ActionIntakeInChamber(intake,horizSlides,outtake);
        actionInertiaSpit = new ActionInertiaSpit(outtake);
    }

    @Override
    public void init_loop() {
        if (alliance != Alliance.RED_ALLIANCE || alliance != Alliance.BLUE_ALLIANCE) {
            telemetry.addData("Alliance:", "None");
        }
        driverOp.readButtons();
        scorerOp.readButtons();
        //alliance:
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_UP) || scorerOp.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//            telemetry.clear();
            alliance = Alliance.RED_ALLIANCE;
        } else if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || scorerOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//            telemetry.clear();
            alliance = Alliance.BLUE_ALLIANCE;
        }
        telemetry.addData("Alliance:", alliance);
        telemetry.update();
    }

    @Override
    public void loop() {
        driverOp.readButtons();
        scorerOp.readButtons();
        telemetry.update();
        updateDebug();

//        colorSensor.showTelemetry(telemetry);
        updateLift();
        updateDrive();
        updateIntake();
        updateOuttake();
        updateHoriz();
        updateActions();
        colorSensor.setINTAKE_HOLD_TRIGGER_DISTANCE(INTAKE_HOLD_TRIGGER_DISTANCE);
    }
    public static double  INTAKE_HOLD_TRIGGER_DISTANCE = 4.0;
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public static double deltaTime = 450;
    public static int TRANSFER_TIMER_0 = 160;
    public static int TRANSFER_TIMER_1 = 1;
    public static int TRANSFER_TIMER_2 = 1;
    public static int TRANSFER_TIMER_3 = 150;
    public static double GEAR_OFFSET = 0.17;
    public static double WRIST_TARGET_TRANSFER = 0;
    public static double SPECI_GEAR_OFFEST = 0.8; //OBSOLETE
// Lift Positions for Specimen
    public static int POSITION_SCORE_1 = 1650;
    public static int POSITION_SCORE_2 = 2265;
    public static int POSITION_SCORE_0 = 0;
//  Positions for Specimens
    public static double SPECI_GEAR_POS_0 = 0.8;
    public static double SPECI_GEAR_POS_1 = 0.13;
    public static double SPECI_WRIST_POS_0_OFFSET = -0.63;
    public static double SPECI_WRIST_POS_1 = 0.07;
    public static int SPECI_HANG_GEAR_AFTER_RELEASE_TIMER = 10;

// outtake positions
    public static double GEAR_MIDDLE = 0.5;
    public static double GEAR_TARGET_INERTIA_1 = 0.35;
    public static double GEAR_TARGET_INERTIA_2 = 0.8;
    public static double GEAR_TARGET_TRANSFER = 0.11;
    public static double GEAR_TARGET_HB = 0.58;
    public static int INERTIA_TIMER_1 = 50;
    private void updateActions() {
        actionTransfer.update(scorerOp, telemetry, getBatteryVoltage());
        actionTransfer.updateConstants(GEAR_OFFSET, GEAR_MIDDLE, GEAR_TARGET_INERTIA_1, GEAR_TARGET_INERTIA_2, TRANSFER_TIMER_0, TRANSFER_TIMER_1, TRANSFER_TIMER_2, TRANSFER_TIMER_3, GEAR_TARGET_TRANSFER, GEAR_TARGET_HB, INERTIA_TIMER_1, WRIST_TARGET_TRANSFER);
//        actionTransfer.updateDeltaTime(deltaTime);
        actionSpecimens.update(scorerOp, telemetry, getBatteryVoltage());
        actionSpecimens.setSPECI_GEAR_OFFEST(SPECI_GEAR_OFFEST);
        actionSpecimens.setConstants(POSITION_SCORE_1, POSITION_SCORE_2, POSITION_SCORE_0, SPECI_WRIST_POS_0_OFFSET, SPECI_GEAR_POS_0, GEAR_MIDDLE, SPECI_GEAR_POS_1, SPECI_WRIST_POS_1, SPECI_HANG_GEAR_AFTER_RELEASE_TIMER);
        //
        actionScoreHighBasket.update(scorerOp, telemetry, getBatteryVoltage());
        actionIntakeInChamber.update(scorerOp, driverOp, telemetry, getBatteryVoltage(), actionTransfer);
        actionInertiaSpit.update(scorerOp, telemetry, getBatteryVoltage());
    }

//    public void runTransferOnce(){
//        actionTransfer.runOnce();
//    }

    public static double leftArmTarget = 0;
    public static double rightArmTarget = 0;
    public static double LEFT_POPRAV_TIME = 0.5;
    public static double RIGHT_POPRAV_TIME = 0.5;
    public static double LEFT_POPRAV_POWER_1 = 0.5;
    public static double RIGHT_POPRAV_POWER_1 = -0.5;
    public static double LEFT_POPRAV_POWER_2 = 0.5;
    public static double RIGHT_POPRAV_POWER_2 = -0.5;
    public static double INTAKE_ARM_LOWER_LIMI = 0;
    private void updateIntake() {
//        intake.updateIntake(scorerOp, correctColor);
//        intake.updateTargets();
//        intake.updateIntake(scorerOp, getBatteryVoltage());
        intake.updateIntake(getBatteryVoltage());
        intake.setArmLowerLimit(INTAKE_ARM_LOWER_LIMI);
        intake.set_poprav_times(LEFT_POPRAV_TIME, RIGHT_POPRAV_TIME);
        intake.set_poprav_powers(LEFT_POPRAV_POWER_1, RIGHT_POPRAV_POWER_1, LEFT_POPRAV_POWER_2, RIGHT_POPRAV_POWER_2);
    }

    private void updateDebug() {
        if (SYSTEM_IN_DEBUG_MODE) {
            testMecanumWheels();
        }
    }

    private void testMecanumWheels() {
        if (SYSTEM_IN_DEBUG_MODE) {
            if (driverOp.getButton(GamepadKeys.Button.X)) {
                FLMotor.set(0.5);
            }
            if (driverOp.getButton(GamepadKeys.Button.Y)) {
                FRMotor.set(0.5);
            }
            if (driverOp.getButton(GamepadKeys.Button.A)) {
                BLMotor.set(0.5);
            }
            if (driverOp.getButton(GamepadKeys.Button.B)) {
                BRMotor.set(0.5);
            }
        }
    }

    private void updateDrive() {
        double elapsedTime = headingTimer.milliseconds() / 1000.0;
        headingTimer.reset();
        double strafeSpeed = driverOp.getLeftX();
        double forwardSpeed = driverOp.getLeftY();
        double turnInput = driverOp.getRightX();
        double heading = getHeading(imu);
//        telemetry.addData("Strafe1:", strafeSpeed);
//        telemetry.addData("Forward1:", forwardSpeed);
//        telemetry.addData("Turn1:", turnInput);
        if (driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) {
            speedMode = SystemSpeedMode.ULTRASLOW;
        } else {
            speedMode = SystemSpeedMode.DEFAULT;
        }
//        telemetry.addData("System speed mode:", speedMode);
        strafeSpeed = speedMultiplier(strafeSpeed);
        forwardSpeed = speedMultiplier(forwardSpeed);
        turnInput = speedMultiplier(turnInput);
//        telemetry.addData("Strafe2:", strafeSpeed);
//        telemetry.addData("Forward2:", forwardSpeed);
//        telemetry.addData("Turn2:", turnInput);
//        double headingError = heading - headingTarget;
        headingTarget += turnInput * HEADING_TURN_SPEED * elapsedTime;


        if (driverOp.wasJustPressed(GamepadKeys.Button.BACK))
            useFieldCentric = !useFieldCentric;

        if (driverOp.wasJustPressed(GamepadKeys.Button.START)) {
            imu.resetYaw();
//            driveRR.localizer.update()
//            driveRR.
//            driveRR.pose.times(new Pose2d(new Vector2d(0,0),0));
            headingTarget = 0;
        }


        if (driverOp.wasJustPressed(GamepadKeys.Button.X)) {
            if (Math.floor(headingTarget / 90.0) * 90.0 != headingTarget)
                headingTarget = Math.floor(headingTarget / 90.0) * 90.0;
            else
                headingTarget = -90.0;
        }
        if (driverOp.wasJustPressed(GamepadKeys.Button.Y)) {
            headingTarget = 0;
        }
        if(driverOp.wasJustPressed(GamepadKeys.Button.B)){
            if (Math.floor(headingTarget / 90.0) * 90.0 != headingTarget)
                headingTarget = Math.floor(headingTarget / 90.0) * 90.0;
            else
                headingTarget = 90.0;
        }
        if(driverOp.wasJustPressed(GamepadKeys.Button.A)){
            headingTarget = 180;
        }

        if(driverOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
//            if (Math.ceil(headingTarget / 45.0) * 45.0 != headingTarget)
//                headingTarget = Math.floor(headingTarget / 45.0) * 45.0;
//            else
//                headingTarget -= 45.0;
            if(headingTarget <= 90.0 || headingTarget >= -90.0){
                if (Math.floor(headingTarget / 135.0) * 135.0 != headingTarget)
                    headingTarget = Math.floor(headingTarget / 135.0) * 135.0;
                else
                    headingTarget = -135;
            }
        }
        if(driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
//            if (Math.ceil(headingTarget / 45.0) * 45.0 != headingTarget)
//                headingTarget = Math.floor(headingTarget / 45.0) * 45.0 ;
//            else
//                headingTarget += 45.0;
            if (Math.floor(headingTarget / 45.0) * 45.0 != headingTarget)
                headingTarget = Math.floor(headingTarget / 45.0) * 45.0;
            else
                headingTarget = 45.0;
        }

        double turnPower = headingPIDControl(headingTarget, heading);

        if (useFieldCentric) {
            drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnPower, -heading);
        } else {
            // og: strafe, forward, turn
            // strafe <-> turn, forward <-> turn
            drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnInput);
        }
        telemetry.addData("using field centric: ", useFieldCentric);
//        telemetry.addData("strafeSpeed", strafeSpeed);
//        telemetry.addData("forwardSpeed", forwardSpeed);
//        telemetry.addData("turnSpeed", turnInput);
        telemetry.addData("heading: ", heading);
        telemetry.addData("heading target", headingTarget);
        telemetry.addData("heading error", heading - headingTarget);

    }

    public static double gearTarget = 0.5; //0 - back, 1 - forward (towards intake)
    //    public static double gearRtarget = 0.5;
    public static double clawTarget = 1; //0.518 - closed, 1 - open
    public static double wristTarget = 0; // 0.5 - middle, 0 - up, 1 - down

    private void updateOuttake() {
//        outtake.updateTargets(gearTarget, clawTarget, wristTarget);
//        outtake.updateClawTarget(clawTarget);
//        outtake.updateGearTarget(gearTarget);
//        outtake.updateWristTarget(wristTarget);
        outtake.updateOuttake();
    }

    public static double LIFT_KP = 0.0052;
    public static double LIFT_KI = 0.000;
    public static double LIFT_KD = 0.0;
    public static double LIFT_FF = 0.0;
    public static int LIFT_SPEED = 300;
    public static int LIFT_MAX_SAFE_POS = 3500; // 3500
    public static int LIFT_TARGET_HB = 3465; // 3350
    public static int LIFT_MIN_POS = 0;
    public static int LIFT_TARGET = 0;
    public static int LIFT_TOLERANCE = 50;

    private void updateLift() {
        lift.updateLift(scorerOp, telemetry);
//        lift.updateLiftTarget(LIFT_TARGET, LIFT_TOLERANCE);
        lift.updateConstants(LIFT_SPEED, LIFT_MAX_SAFE_POS, LIFT_MIN_POS, LIFT_TARGET_HB);
//        lift.setPIDFFCoefficients(LIFT_KP, LIFT_KI, LIFT_KD, LIFT_FF);
        lift.prostoGiveTelemetry(telemetry);
    }

    public static double HORIZ_KP = 0.0023;
    public static double HORIZ_KI = 0.0;
    public static double HORIZ_KD = 0.0;
    public static int HORIZ_SPEED = 2500;
    public static int HORIZ_MAX_SAFE_LIMIT = 1700;
    public static int HORIZ_MIN_LIMIT = 0;

    public static int HORIZ_TARGET = 0;
    public static double HORIZ_ANTITENSION_POWER_ZERO = -0.2;
    private void updateHoriz() {
//        horizSlides.updateHorizTarget(HORIZ_TARGET, 10);
        horizSlides.updateConstants(HORIZ_SPEED, HORIZ_MAX_SAFE_LIMIT, HORIZ_MIN_LIMIT);
        horizSlides.setHORIZ_ANTITENSION_POWER_ZERO(HORIZ_ANTITENSION_POWER_ZERO);
        horizSlides.updateHoriz(scorerOp, telemetry);
    }

    public double getHeading(BNO055IMUNew imu) {
//        return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double speedMultiplier(double input) {
        double speed = input;
        switch (speedMode) {
            case DEFAULT:
                speed = input;
                break;
            case TURBO:
                speed = Math.pow(input, 1.0 / 3);
                break;
            case SLOW:
                speed = Range.clip(input * input * input, -0.6, 0.6);
                break;
            case ULTRASLOW:
                speed = Range.clip(input * input * input * input * input, -0.3, 0.3);
                break;
        }
        return speed;
    }

    public void updateSystemSpeedMode() {
        if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            switch (speedMode) {
                case DEFAULT:
                    speedMode = SystemSpeedMode.TURBO;
                    break;
                case SLOW:
                    speedMode = SystemSpeedMode.DEFAULT;
                    break;
                case ULTRASLOW:
                    speedMode = SystemSpeedMode.SLOW;
                    break;
            }
        } else if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            switch (speedMode) {
                case DEFAULT:
                    speedMode = SystemSpeedMode.SLOW;
                    break;
                case SLOW:
                    speedMode = SystemSpeedMode.ULTRASLOW;
                    break;
                case TURBO:
                    speedMode = SystemSpeedMode.DEFAULT;
            }
        }
        telemetry.addData("Current speed mode: ", speedMode);
    }

    public double angleWrap(double degrees) {
        while (degrees > 180) {
            degrees -= 360;
        }
        while (degrees < -180) {
            degrees += 360;
        }
        return degrees;
    }

//    public double angleRadWrap(double radians) {
//        while (radians > Math.PI) {
//            radians -= 2 * Math.PI;
//        }
//        while (radians < -Math.PI) {
//            radians += 2 * Math.PI;
//        }
//        return radians;
//    }

    public double headingPIDControl(double reference, double state) {
        double error = angleWrap(reference - state);
//        telemetry.addData("error:", error);
        integralSum += error * headingTimer.seconds();
        double derivative = (error - lastError) / (headingTimer.seconds());
        lastError = error;
        headingTimer.reset();
        return error * HEADING_KP + derivative * HEADING_KD + integralSum * HEADING_KI;
    }

}
