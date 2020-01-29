package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.roadrunner.mecanum.SampleMecanumDriveREV;
import org.roadrunner.util.AssetsTrajectoryLoader;

import java.io.IOException;

@Autonomous()
//@Disabled
public class TrajectoryTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Trajectory traj = null;
        try {
            traj = AssetsTrajectoryLoader.load("SimpleTest") ;
        } catch (IOException e) {
            e.printStackTrace();
        }


        waitForStart();

        if (isStopRequested()) return;

        if (traj != null) {
            SampleMecanumDriveREV drive = new SampleMecanumDriveREV(hardwareMap);
            drive.followTrajectorySync(traj);
        }

    }
}
