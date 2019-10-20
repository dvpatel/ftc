package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Target Drive")
//  @Disabled
public class TargetDrive extends AbstractLinearOpMode {

    private MotorControllerEx motor;
    private Driver driver;
    private IMUController imu;

    @Override
    void initOpMode() throws InterruptedException {

        this.initRosie();

        this.imu = this.rosie.getIMUController();
        this.motor = this.rosie.getMotorPID();
        this.driver = this.rosie.getDriver();

        //  Enable PID Controller to track state
        //  this.motor.enablePID();
        //this.motor.enableDrivePID(power);

        telemetry.addData("Mode", "init complete;  Running");
        telemetry.update();


    }

    @Override
    void stopOpMode() {
        driver.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //  Put common init logic here
        this.initOpMode();

        //  Activate opmode
        this.waitToPressStart();

        //  make sure power is between -1 and 1 ;
        double power = this.normalizePower(0.3);

        while (opModeIsActive()) {

            //  Drive forward 3 inches ;
            double distance = 3;
            telemetry.addData("GyroDrive:  ", "drive...");
            telemetry.update();
            this.drive(distance, power);
            telemetry.addData("GyroDrive:  ", "Done..");
            telemetry.update();
            sleep(5000);

            //  Strafe 3 inches left ;
            telemetry.addData("GyroStrafe:  ", "strafe left");
            telemetry.update();
            distance = -3;
            this.strafe(distance, -power);
            telemetry.addData("GyroStrafe:  ", "Done..");
            telemetry.update();
            sleep(5000);


            //  drive backward 3 inches ;
            telemetry.addData("GyroDrive:  ", "back 3 inches");
            telemetry.update();
            distance = -3;
            this.drive(distance, -power);
            telemetry.addData("GyroDrive:  ", "Done..");
            telemetry.update();
            sleep(5000);


            //  Strafe 3 inches right ;
            telemetry.addData("GyroStrafe:  ", "strafe 3 inches right");
            telemetry.update();
            distance = 3;
            this.strafe(distance, power);
            telemetry.addData("GyroDrive:  ", "Done..");
            telemetry.update();
            sleep(5000);

            //  Turn 90 degrees to the right
            telemetry.addData("GyroTurn:  right ", "180 degrees");
            telemetry.update();
            double degrees = 180;
            this.turn(degrees, power);
            telemetry.addData("GyroTurn:  Done, ", imu.getAngle());
            telemetry.update();
            sleep(5000);


            telemetry.addData("GyroTurn: left  ", "180 degrees??");
            telemetry.update();
            degrees = -180;
            this.turn(degrees, -power);
            telemetry.addData("GyroTurn:  Done, ", imu.getAngle());
            telemetry.update();
            sleep(5000);
        }

        this.stopOpMode();
    }
}
