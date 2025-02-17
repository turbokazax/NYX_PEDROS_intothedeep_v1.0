package yori.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GamepadRumbler {
    private static boolean wasGamepadRumbled = false;
    private static ElapsedTime driverOpTimer = new ElapsedTime();
    private static ElapsedTime scorerOpTimer = new ElapsedTime();
    public static void rumbleOnce(GamepadEx gamepadEx, int durationMS){
        if(gamepadEx.gamepad == gamepad1){
            gamepadEx.gamepad.rumble(durationMS);
            if(driverOpTimer.milliseconds() >= durationMS){
                gamepadEx.gamepad.stopRumble();
            }
        }
        if(gamepadEx.gamepad == gamepad2){
            gamepadEx.gamepad.rumble(durationMS);
            if(scorerOpTimer.milliseconds() >= durationMS){
                gamepadEx.gamepad.stopRumble();
                scorerOpTimer.reset();
            }
        }

    }
}
