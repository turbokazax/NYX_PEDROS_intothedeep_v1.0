/**
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
import yori.AUTOOO.actions_auto.ActionScoreHighBasket;
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

    public static double poses[][] = {
            {0,0},
            {0,0},
            {0,0},
            {0,0},
            {0,0},
            {0,0},
            {0,0},
            {0,0},
            {0,0},
            {0,0},
            {0,0}

    };


    private final Pose startPose = new Pose(poses[0][0],poses[0][1], 0);
    private final Pose score = new Pose(poses[1][0], poses[1][1], -45);
    private final Pose take1 = new Pose(poses[2][0], poses[2][1],0);
    private final Pose take2 = new Pose(poses[3][0],poses[3][1],0);
    private final Pose take3 = new Pose(poses[4][0],poses[4][1],0);
    private final Pose control = new Pose(poses[5][0],poses[5][1]);
    private final Pose park = new Pose(poses[6][0], poses[6][1], 90);


    private Path starting, taking1, scoring1, taking2, scoring2, taking3, scoring3, parking;

    public void buildPaths() {

        starting = new Path(new BezierLine(startPose, score));
        starting.setLinearHeadingInterpolation(startPose.getHeading(), score.getHeading());

        taking1 = new Path(new BezierLine(score, take1));
        taking1.setLinearHeadingInterpolation(score.getHeading(), take1.getHeading());

        scoring1 = new Path(new BezierLine(take1, score));
        scoring1.setLinearHeadingInterpolation(take1.getHeading(), score.getHeading());

        scoring2 = new Path(new BezierLine(take2, score));
        scoring2.setLinearHeadingInterpolation(take2.getHeading(), score.getHeading());

        scoring3 = new Path(new BezierLine(take3, score));
        scoring3.setLinearHeadingInterpolation(take3.getHeading(), score.getHeading());

        taking2 = new Path(new BezierLine(score, take2));
        taking2.setLinearHeadingInterpolation(score.getHeading(), take2.getHeading());

        taking3 = new Path(new BezierLine(score, take3));
        taking3.setLinearHeadingInterpolation(score.getHeading(), take3.getHeading());


        parking = new Path(new BezierCurve(score,control,park));
        parking.setLinearHeadingInterpolation(score.getHeading(),park.getHeading());

    }

    public static int pathState = 0;

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(starting, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy())
                    follower.followPath(taking1, true);
                    ActionTransfer.
//                    setPathState(2);
                break;
            case 2:
                follower.followPath(scoring1, true);
                setPathState(3);
                break;
            case 3:
                follower.followPath(taking2, true);
                setPathState(4);
                break;
            case 4:
                follower.followPath(scoring2, true);

                setPathState(5);
                break;
            case 5:
                follower.followPath(taking3, true);

                setPathState(6);
                break;
            case 6:
                follower.followPath(scoring3, true);

                setPathState(7);
                break;
            case 7:
                follower.followPath(parking, true);

                setPathState(-1);
                break;
        }
    }
    @Override
    public void loop() {
        autonomousPathUpdate();
        updateActions();
        updateMechas();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        follower.telemetryDebug(telemetry);

        telemetry.update();
    }

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
**/