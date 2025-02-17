package yori.AUTOOO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import yori.AUTOOO.actions_auto.ActionIntakeInChamber;
import yori.AUTOOO.actions_auto.ActionTransfer;
import yori.mechas.HorizSlides;
import yori.mechas.Intake;
import yori.mechas.Outtake;

@Autonomous(name = "TEST")
public class TEST extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;

    // Mechas
    private Intake intake;
    private HorizSlides horizSlides;
    private Outtake outtake;

    // Actions
    private ActionTransfer actionTransfer;
    private ActionIntakeInChamber actionIntakeInChamber;

    /** Define Poses **/
    private final Pose startPose = new Pose(8.354, 101.635, Math.toRadians(0));
    private final Pose midPose = new Pose(24.663, 120.133, Math.toRadians(0));
    private final Pose finalPose = new Pose(14.320, 130.475, Math.toRadians(0));

    /** Define Paths and PathChains **/
    private Path path1;
    private PathChain line2;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        pathTimer = new Timer();
        pathTimer.resetTimer();
        actionTimer = new Timer();
        actionTimer.resetTimer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap);
        horizSlides = new HorizSlides(hardwareMap);
        actionTransfer = new ActionTransfer(outtake, intake, horizSlides);
        actionIntakeInChamber = new ActionIntakeInChamber(intake, horizSlides, outtake);

        buildPaths();
    }

    /** Build the paths for the auto **/
    public void buildPaths() {
        path1 = new Path(new BezierLine(new Point(startPose), new Point(midPose)));
        path1.setLinearHeadingInterpolation(startPose.getHeading(), midPose.getHeading());

        line2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(midPose), new Point(finalPose)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();
    }

    public static int pathState = 0;

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    actionIntakeInChamber.setSequenceState(ActionIntakeInChamber.SequenceState.HORIZ_MID);
                    if (actionTransfer.isTransferRunning()) {
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(line2, true);
                }
                break;
        }
    }

    @Override
    public void loop() {
        autoPathUpdate();
        updateActions();
        updateMechas();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    private void updateActions() {
        actionTransfer.update(telemetry, getBatteryVoltage());
        actionIntakeInChamber.update(telemetry, getBatteryVoltage(), actionTransfer);
    }

    private void updateMechas(){
        intake.updateIntake(getBatteryVoltage());
        outtake.updateOuttake();
        horizSlides.updateHoriz_AUTO(telemetry);
    }

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
}
