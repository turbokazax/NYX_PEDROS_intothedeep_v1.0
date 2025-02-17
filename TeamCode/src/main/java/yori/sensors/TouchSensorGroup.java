package yori.sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TouchSensorGroup {
    private DigitalChannel touchLeft;
    private DigitalChannel touchRight;

    public TouchSensorGroup(HardwareMap hw){
        touchLeft = hw.get(DigitalChannel.class, "touch_left");
        touchRight = hw.get(DigitalChannel.class, "touch_right");
        touchLeft.setMode(DigitalChannel.Mode.INPUT);
        touchRight.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isLeftTouchPressed(){
        return (touchLeft.getState() == false);
    }
    public boolean isRightTouchPressed(){
        return (touchRight.getState() == false);
    }

    public boolean isLiftAtMinPos(){
        //TODO: vozmojno pomenyat na stogiy AND
        return isLeftTouchPressed() || isRightTouchPressed();
    }

    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("LEFT_TOUCH", isLeftTouchPressed());
        telemetry.addData("RIGHT_TOUCH", isRightTouchPressed());
    }

}
