package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.00295038763102323934;
        TwoWheelConstants.strafeTicksToInches = 0.0029427015256177154;
        TwoWheelConstants.forwardY = 0.15346457; // in, 3.898000078 mm
        TwoWheelConstants.strafeX = -6.38484252; //in, 162.175000008 mm
        TwoWheelConstants.forwardEncoder_HardwareMapName = "FLmotor"; // expansion, port 0, M (par)
        TwoWheelConstants.strafeEncoder_HardwareMapName = "perp"; // expansion, port 3, B (perp)
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
    }
}




