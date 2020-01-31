package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.Constants;
import org.blueprint.ftc.core.IntakeSystem;
import org.blueprint.ftc.core.SkystoneDetector;
import org.blueprint.ftc.core.TrajectoryFactory;
import org.roadrunner.DriveConstants;
import org.roadrunner.mecanum.SampleMecanumDriveREV;

import java.io.IOException;


//  See
//  https://github.com/gearsincorg/FTCVuforiaDemo/blob/master/TeleopOpmode.java

@Autonomous(name = "SkystoneDetection")
//  @Disabled
public class SkystoneDetectionTele extends BaseAutonomous {

    private SkystoneDetector skystoneDetector;
    private IntakeSystem intakeSystem;

    private boolean isBlue;

    double[] targetCoordinates;
    int detectedStoneNumber;

    //  Game quadrant;
    private GameQuadrant quadrant;

    private boolean quadrantSelected;


    @Override
    public void initOpMode() throws InterruptedException {
        this.initRosie();

        this.intakeSystem = this.rosie.getIntakeSystem();

        //  this.detectSkystoneOnField();
        this.skystoneDetector = this.rosie.getSkystoneDetector();

        this.setupZone();

        detectedStoneNumber = this.skystoneDetector.detectSkystoneOnField(this, isBlue);
    }

    private void setupZone() throws InterruptedException{
        //  Select quadrant;
        this.quadrant = this.selectGameQuadrant();
        telemetry.addData("Selected Zone:  ", quadrant.toString());
        telemetry.addData("Robot Direction (- left, + right):  ", GameQuadrant.direction(quadrant));
        telemetry.addData("Mode:  ", "init complete;  Running");
        telemetry.update();

        switch(quadrant) {
            case LOADING_RED:
                isBlue = false;
                break;
            case LOADING_BLUE:
                isBlue = true;
                break;
            case BUILDING_BLUE:
            case BUILDING_RED:
                throw new InterruptedException("Robot cannot be in building zone.  Start again.");
        }
    }

    @Override
    public void stopOpMode() {
        this.intakeSystem.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.initOpMode();

        telemetry.addData("Stone Number", detectedStoneNumber);
        telemetry.update();

        this.waitToPressStart();

        //  Intake system too high, also can't be down before start of match or in Init.  Refs don't like this.
        this.intakeSystem.setIntakeServosInitPosition();

        //  Force value
        detectedStoneNumber = 1;

        if (opModeIsActive()) {

            if (isBlue) {
                //  this.blueMission(detectedStoneNumber);
            } else {
                //  this.redMission(detectedStoneNumber);
            }


            //  this.goToBlueStone(detectedStoneNumber);
            //  this.pickupBlueStone();


            //  over the stone
            //  this.intakeSystem.start();
            //  move down inch?
        }

        this.stopOpMode();
    }

    private void blueMission(int stoneNumber) {
        String trajName = "BlueLoadingStone"+stoneNumber;
        this.goToStone(trajName);

        //  Next step;
    }

    private void redMission(int stoneNumber){
        String trajName = "RedLoadingStone"+stoneNumber;
        this.goToStone(trajName);

        //  Next step;
    }

    private void goToStone(String trajName) {
        try {
            SampleMecanumDriveREV drive = new SampleMecanumDriveREV(hardwareMap);
            Trajectory traj = TrajectoryFactory.getInstance().getTrajectory(trajName);
            drive.followTrajectorySync(traj);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void pickupBlueStone() {
        this.intakeSystem.start();
        this.driveReverse(2);  //  2 inches;
        this.intakeSystem.stop();
    }

    //  Not dynamic trajectory;
    private void blueLoadingStone() {

        double dX = Math.abs(targetCoordinates[0]) - 2.25;
        double dY = targetCoordinates[1];

        SampleMecanumDriveREV drive = new SampleMecanumDriveREV(hardwareMap);

        BaseTrajectoryBuilder baseTraj = drive.trajectoryBuilder()
                .forward(dX);

        if (dY > 0) {
            baseTraj = baseTraj.strafeRight(Math.abs(dY) + Constants.DRIVETRAIN_WIDTH);
        } else {
            baseTraj = baseTraj.strafeLeft(Math.abs(dY) + Constants.DRIVETRAIN_WIDTH);
        }

        Trajectory traj = baseTraj.build();
        drive.followTrajectorySync(traj);

    }

    private void blueLoadingStoneLast() {
        SampleMecanumDriveREV drive = new SampleMecanumDriveREV(hardwareMap);

        double dX = 24.0 + (24.0 - Constants.DRIVETRAIN_LENGTH);
        double dY = 20.0 + (Constants.DRIVETRAIN_WIDTH / 2);

        Trajectory traj = drive.trajectoryBuilder().forward(dX).strafeRight(dY).build();
        drive.followTrajectorySync(traj);
    }
}
