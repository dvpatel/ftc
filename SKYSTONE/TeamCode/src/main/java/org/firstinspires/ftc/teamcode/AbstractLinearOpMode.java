package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

public abstract class AbstractLinearOpMode extends LinearOpMode {

    abstract void initOpMode() throws InterruptedException;

    abstract void stopOpMode();

    protected GameBot rosie;

    protected void waitToPressStart() {
        // wait for start button.
        waitForStart();

        //  why is this needed?
        sleep(1000);
    }

    protected void initRosie() throws InterruptedException {
        this.rosie = new GameBot();
        this.rosie.init(hardwareMap);
    }

    protected double normalizePower(double power) {
        return Range.clip(power, -1.0, 1.0);
    }

    protected void stopDriving() {
        this.rosie.getDriver().stop();
    }

    protected void drive(double power) {
        Driver driver = this.rosie.getDriver();
        this.gyroDrive(power);
    }

    protected void drive(double distanceInInches, double power) {

        Driver driver = this.rosie.getDriver();

        driver.setTargetPosition(distanceInInches);
        this.gyroDrive(power);
        while (opModeIsActive() && driver.motorsBusy()) {
            telemetry.addData("GyroDrive:  ", "driving...");
            this.gyroDrive(power);
            telemetry.update();
        }
        driver.stop();
    }

    protected void strafe(double power) {
        Driver driver = this.rosie.getDriver();
        this.gyroStrafe(power);
    }

    protected void turn(double degrees, double power) {

        Driver driver = this.rosie.getDriver();
        MotorControllerEx motor = this.rosie.getMotorPID();
        IMUController imu = this.rosie.getIMUController();

        imu.resetAngle();

        do {
            telemetry.addData("GyroTurn:  ", "turning...");
            double[] p = motor.calculateRotateCorrection(degrees, imu.getAngle(), power);

            if (degrees < 0) {
                driver.rotateDifferential(p[0], -p[1]);
            } else {
                driver.rotateDifferential(-p[0], p[1]);
            }

            telemetry.update();
        } while (opModeIsActive() && !motor.angleOnTarget());

        driver.stop();
    }

    protected void strafe(double distanceInInches, double power) {

        Driver driver = this.rosie.getDriver();
        driver.setTargetPosition(distanceInInches);
        this.gyroStrafe(power);
        while (opModeIsActive() && driver.motorsBusy()) {
            telemetry.addData("GyroStrafe:  ", "strafing...");
            this.gyroStrafe(power);
            telemetry.update();
        }
        driver.stop();
    }

    protected void gyroDrive(double power) {

        Driver driver = this.rosie.getDriver();
        MotorControllerEx motor = this.rosie.getMotorPID();
        IMUController imu = this.rosie.getIMUController();

        //  index 0:  leftPowerCorrection
        //  index 1:  rightPowerCorrection
        //  index 2:  correction value ;
        double[] correction = motor.calculateDriveCorrection(power, imu.getAngle());
        boolean showDebug = correction[0] != 0 || correction[1] != 0;

        driver.driveDifferential(this.normalizePower(correction[0]), this.normalizePower(correction[1]));

        if (showDebug) {
            telemetry.addData("Non-Zero Angle.  Adjust power.", imu.getAngle());
            telemetry.update();
        }
    }

    //  positive power strafe left; negative strafe right
    protected void gyroStrafe(double power) {

        Driver driver = this.rosie.getDriver();
        MotorControllerEx motor = this.rosie.getMotorPID();
        IMUController imu = this.rosie.getIMUController();

        //  index 0:  leftPowerCorrection
        //  index 1:  rightPowerCorrection
        //  index 2:  correction value ;
        double[] correction = motor.calculateDriveCorrection(power, imu.getAngle());
        boolean showDebug = correction[0] != 0 || correction[1] != 0;

        driver.strafeDifferential(this.normalizePower(correction[0]), this.normalizePower(correction[1]));

        if (showDebug) {
            telemetry.addData("Non-Zero Angle.  Adjust power.", imu.getAngle());
            telemetry.update();
        }
    }
}

