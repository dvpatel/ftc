package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Target Drive")
@Disabled
public class TargetDrive extends AbstractBaseLinearOpMode {

    private MotorControllerEx motor;
    private Driver driver;
    private IMUController imu;

    private double angleCorrection;
    private double power = 0.30;

    @Override
    void initRobot() {

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
    void stopRobot() {
        driver.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //  Put common init logic here
        this.initRosie();
        this.initRobot();

        //  Activate opmode
        this.waitToPressStart();

        //  Drive forward 3 inches ;
        telemetry.addData("GyroDrive:  ", "forward 3 inches");
        telemetry.update();

        this.driver.setTargetPositionInInches(3);
        this.gyroDrive(power);
        while (opModeIsActive() && this.driver.motorsBusy()) {
            telemetry.addData("GyroDrive:  ", "driving...");
            this.gyroDrive(power);
            telemetry.update();
        }

        this.stop();

        telemetry.addData("GyroDrive:  ", "Done..");
        telemetry.update();

        //  Sleep.
        sleep(5000);

        //  drive backward 3 inches ;
        //  Drive forward 3 inches ;
        telemetry.addData("GyroDrive:  ", "forward -3 inches");
        telemetry.update();

        this.driver.setTargetPositionInInches(-3);
        this.gyroDrive(-power);
        while (opModeIsActive() && this.driver.motorsBusy()) {
            telemetry.addData("GyroDrive:  ", "driving...");
            this.gyroDrive(-power);
            telemetry.update();
        }
        this.stop();

        telemetry.addData("GyroDrive:  ", "Done..");
        telemetry.update();

    }

    private void gyroDrive(double power) {
        //  index 0:  leftPowerCorrection
        //  index 1:  rightPowerCorrection
        //  index 2:  correction value ;
        double[] correction = this.motor.calculateDriveCorrection(power, this.imu.getAngle());
        boolean showDebug = correction[0] != 0 || correction[1] != 0;

        driver.driveDifferential(correction[0], correction[1]);
        angleCorrection = correction[2];

        if (showDebug) {
            telemetry.addData("Non-Zero Angle.  Adjust power.", this.imu.getAngle());
            telemetry.update();
        }
    }

}
