package yori.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import yori.sensors.ColorSensor;
import yori.utils.Alliance;

@TeleOp(name = "Color Sensor Testing", group = "Testing")
public class ColorSensorTestingOP extends OpMode {
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    ColorSensor sensor;
    Alliance alliance;
    @Override
    public void init() {
        sensor = new ColorSensor(hardwareMap);
        alliance = Alliance.RED_ALLIANCE;
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
//        telemetry.addData("Color?", sensor.updateColorSensor(alliance));
        sensor.updateColorSensor(alliance);
        sensor.showTelemetry(telemetry);
    }
}
