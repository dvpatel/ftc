package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.blueprint.ftc.core.TrajectoryFactory;
import org.roadrunner.mecanum.SampleMecanumDriveREV;
import org.roadrunner.util.AssetsTrajectoryLoader;

import java.io.IOException;

@Config
@Autonomous()
@Disabled
public class TrajectoryTest extends LinearOpMode {

    /*
      "BlueFoundation", "BlueLoadingStone1", "BlueLoadingStone2", "BlueLoadingStone3",
      "RedFoundation", "RedLoadingStone1", "RedLoadingStone2", "RedLoadingStone3";
     */

    public static String TRAJECTORY_NAME = "RedLoadingStone3";

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDriveREV drive = new SampleMecanumDriveREV(hardwareMap);


        //  start pose, start heading;
        //  Blue starting position
        //drive.setPoseEstimate(new Pose2d(-32.88, 63.13, 0.0));

        //  Red starting position;
        //drive.setPoseEstimate(new Pose2d(-32.88, -63.13, (Math.PI/2)));

        waitForStart();

        if (isStopRequested()) return;

        try {

            // x, y, heading;
            //  Starting pose;
            //  drive.setPoseEstimate(new Pose2d(-32.88, -63.13, -4.101523742186674));

            Trajectory traj = TrajectoryFactory.getInstance().getTrajectory(TRAJECTORY_NAME);
            drive.followTrajectorySync(traj);
        } catch (IOException e) {
            e.printStackTrace();
        }


    }
}
