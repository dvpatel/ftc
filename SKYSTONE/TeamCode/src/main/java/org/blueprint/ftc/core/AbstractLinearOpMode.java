package org.blueprint.ftc.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

public abstract class AbstractLinearOpMode extends LinearOpMode {

    protected GameBot rosie;

    protected abstract void initOpMode() throws InterruptedException;

    protected abstract void stopOpMode();

    protected void initRosie() throws InterruptedException {
        this.rosie = new GameBot();
        this.rosie.init(hardwareMap);
    }

    protected void waitToPressStart() {
        // wait for start button.
        waitForStart();

        //  why is this needed?
        sleep(1000);
    }

    protected double normalizePower(double power) {
        return Range.clip(power, -1.0, 1.0);
    }

    protected void stopDriving() {
        this.rosie.getDriver().stop();
    }


    protected void driveForward(int distance, double power) {
        this.drive(distance, power);
    }

    protected void driveReverse(int distance, double power) {
        //  Both power and distance needs to be negative to go reverse
        this.drive(-distance, -power);
    }

    protected void strafeLeft(int distance, double power) {
        this.strafe(-distance, -power);
    }

    protected void strafeRight(int distance, double power) {
        //  Note:  both values must be negative ;
        this.strafe(distance, power);
    }

    protected void turnRight(int degrees, double power) {
        this.turn(degrees, power);
    }

    protected void turnLeft(int degrees, double power) {
        //  note degrees direction and power
        this.turn(-degrees, power);
    }

    protected void drive(double power) {
        Driver driver = this.rosie.getDriver();
        this.gyroDrive(power);
    }

    //  GyroDrive
    //  Reverse:  this.drive(-distance, -power);
    protected void drive(double distanceInInches, double power) {

        Driver driver = this.rosie.getDriver();
        driver.setTargetPosition(distanceInInches);
        driver.drive(power);
        while (driver.motorsBusy()) {
            //  leftFront, rightFront, leftBack, rightBack;
            int[] cp = driver.getCurrentPosition();
            telemetry.addData("ticks", cp[0] + ", " + cp[1] + ", " + cp[2] + ", " + cp[3] + ", isBusy=" + driver.motorsBusy());
            telemetry.update();
        }
        driver.stop();

        //  Disable encoders ;
        driver.turnOffEncoders();
    }

    protected void spin(double power) {
        Driver driver = this.rosie.getDriver();
        driver.spin(power);
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

        while (!motor.angleOnTarget()) {
            //  NOTE:  imu.getAngle returns positive on left turn, negative on right turn
            //  Negative degrees means left turn
            double p = motor.calculateRotateCorrection(-degrees, imu.getAngle(), power);
            driver.powerDifferential(-p, p, -p, p);
            telemetry.addData("Power", p);
            telemetry.addData("IMU Angle", imu.getAngle());
            telemetry.update();
        }

        driver.stop();

        driver.turnOffEncoders();
        //  imu.resetAngle();

    }

    protected void strafe(double distanceInInches, double power) {

        Driver driver = this.rosie.getDriver();
        driver.setStrafeTargetPosition(distanceInInches);
        this.rosie.getIMUController().resetAngle();

        driver.strafeDifferential(power, power);
        do {
            int[] cp = driver.getCurrentPosition();
            telemetry.addData("ticks", cp[0] + ", " + cp[1] + ", " + cp[2] + ", " + cp[3] + ", isBusy=" + driver.motorsBusy());
            telemetry.update();
        } while (opModeIsActive() && driver.motorsBusy());

        driver.stop();

        driver.turnOffEncoders();
    }

    protected void gyroDrive(double power) {

        Driver driver = this.rosie.getDriver();
        MotorControllerEx motor = this.rosie.getMotorPID();
        IMUController imu = this.rosie.getIMUController();

        //  index 0:  leftPowerCorrection
        //  index 1:  rightPowerCorrection
        //  index 2:  correction value ;
        double[] p = motor.calculateDriveCorrection(power, imu.getAngle());
        driver.driveDifferential(this.normalizePower(p[0]), this.normalizePower(p[1]));
    }

    //  positive power strafe left; negative strafe right
    protected void gyroStrafe(double power) {

        Driver driver = this.rosie.getDriver();
        MotorControllerEx motor = this.rosie.getMotorPID();
        IMUController imu = this.rosie.getIMUController();

        //  index 0:  leftPowerCorrection
        //  index 1:  rightPowerCorrection
        //  index 2:  correction value ;
        double[] p = motor.calculateDriveCorrection(power, imu.getAngle());
        boolean showDebug = p[0] != 0 || p[1] != 0;

        driver.strafeDifferential(this.normalizePower(p[0]), this.normalizePower(p[1]));

        if (showDebug) {
            telemetry.addData("Non-Zero Angle.  Adjust power.", imu.getAngle());
            telemetry.update();
        }
    }
}

