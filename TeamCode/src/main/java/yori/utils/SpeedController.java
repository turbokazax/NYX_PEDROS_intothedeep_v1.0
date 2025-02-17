package yori.utils;

import com.qualcomm.robotcore.util.Range;

public class SpeedController {
    public enum SystemSpeedMode {
        TURBO,
        DEFAULT,
        SLOW,
        ULTRASLOW
    }

    public SystemSpeedMode speedMode = SystemSpeedMode.DEFAULT;
    public double speedMultiplier(double input) {
        double speed = input;
        switch (speedMode) {
            case DEFAULT:
                speed = Math.signum(speed) * input * input;
                break;
            case TURBO:
                speed = Math.pow(input, 1.0 / 3);
                break;
            case SLOW:
                speed = Range.clip(input * input * input, -0.6, 0.6);
                break;
            case ULTRASLOW:
                speed = Range.clip(input * input * input * input * input, -0.2, 0.2);
                break;
        }
        return speed;
    }
}
