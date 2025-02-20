package yori.sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import yori.utils.Alliance;

public class ColorSensor {
    private NormalizedColorSensor colorSensor;
    private float gain;
    private final float[] hsvValues = new float[3];
    private final float[] rgbValues = new float[3]; // red, green, blue

    private double distance;
    private NormalizedRGBA colors;
    private yori.utils.Color currentColor;
    private boolean correctColor;

    public ColorSensor(HardwareMap hw) {
        colorSensor = hw.get(NormalizedColorSensor.class, "sensor_color");
        gain = 10.3F;
        colorSensor.setGain(gain);
    }

    public void setGain(float gain) {
        this.gain = gain;
        colorSensor.setGain(gain);
    }

    public void showTelemetry(Telemetry telemetry) {
//        telemetry.addData("Red", colors.red);
//        telemetry.addData("Green", colors.green);
//        telemetry.addData("Blue", colors.blue);
//        telemetry.addData("Hue", hsvValues[0]);
//        telemetry.addData("Saturation", hsvValues[1]);
//        telemetry.addData("Value", hsvValues[2]);
//        telemetry.addData("Alpha", colors.alpha);
        telemetry.addData("Current color", currentColor);
        telemetry.addData("Correct color-alliance?", correctColor);
        telemetry.addData("Distance", distance);
    }

    private yori.utils.Color getColors() {
        colors = colorSensor.getNormalizedColors();
        rgbValues[0] = colors.red;
        rgbValues[1] = colors.green;
        rgbValues[2] = colors.blue;
        Color.colorToHSV(colors.toColor(), hsvValues);
        currentColor = getMostPrevalentColor();
        return getMostPrevalentColor();
    }

    public boolean updateColorSensor(Alliance alliance) {
//        getColors();
        colors = colorSensor.getNormalizedColors();
        rgbValues[0] = colors.red;
        rgbValues[1] = colors.green;
        rgbValues[2] = colors.blue;
        Color.colorToHSV(colors.toColor(), hsvValues);
        currentColor = getMostPrevalentColor();
        if (alliance == Alliance.RED_ALLIANCE && currentColor == yori.utils.Color.RED) {
            correctColor = true;
            return true;
        } else if (alliance == Alliance.BLUE_ALLIANCE && currentColor == yori.utils.Color.BLUE) {
            correctColor = true;
            return true;
        } else if (currentColor == yori.utils.Color.YELLOW) {
            correctColor = true;
            return true;
        }
        correctColor = false;
//        showTelemetry(telemetry);
        return false;
    }

    private double INTAKE_HOLD_TRIGGER_DISTANCE = 4.0;

    public void setINTAKE_HOLD_TRIGGER_DISTANCE(double INTAKE_HOLD_TRIGGER_DISTANCE) {
        this.INTAKE_HOLD_TRIGGER_DISTANCE = INTAKE_HOLD_TRIGGER_DISTANCE;
    }

    public double getDistance() {
        return distance;
    }

//    private yori.utils.Color getMostPrevalentColor() {
//        double max = 0;
//        int maxIndex = 0;
//        for (int i = 0; i < 3; i++) {
//            if (rgbValues[i] > max) {
//                max = rgbValues[i];
//                maxIndex = i;
//            }
//        }
//        distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
//        if (distance <= INTAKE_HOLD_TRIGGER_DISTANCE) {
//            if (maxIndex == 0) {
//                return yori.utils.Color.RED;
//            } else if (maxIndex == 1) {
//                return yori.utils.Color.YELLOW;
//            } else {
//                return yori.utils.Color.BLUE;
//            }
//        } else {
//            return yori.utils.Color.UNKNOWN;
//        }
//    }
    private yori.utils.Color getMostPrevalentColor(){
        distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        if (distance <= INTAKE_HOLD_TRIGGER_DISTANCE) {
            if (hsvValues[0] >= 0 && hsvValues[0] < 30 || hsvValues[0] >= 330 && hsvValues[0] < 360) {
                return yori.utils.Color.RED;
            } else if (hsvValues[0] >= 45 && hsvValues[0] <= 120) {
                return yori.utils.Color.YELLOW;
            } else if (hsvValues[0] >= 180 && hsvValues[0] <= 270) {
                return yori.utils.Color.BLUE;
            } else {
                return yori.utils.Color.UNKNOWN;
            }
        } else {
            return yori.utils.Color.UNKNOWN;
        }
    }

}
