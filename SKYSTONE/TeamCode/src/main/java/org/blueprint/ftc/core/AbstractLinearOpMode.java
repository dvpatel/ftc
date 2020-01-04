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
    }

    protected double normalizePower(double power) {
        return Range.clip(power, Constants.DEFAULT_MIN_POWER, Constants.DEFAULT_MAX_POWER);
    }

    protected void stopDriving() {
        this.rosie.getDriver().stop();
    }

    protected void driveForward(double distance, double power) {
        this.drive(distance, power);
    }

    protected void driveReverse(double distance, double power) {
        //  Both power and distance needs to be negative to go reverse
        this.drive(-distance, -power);
    }

    protected void strafeLeft(double distance, double power) {
        this.strafe(-distance, -power);
    }

    protected void strafeRight(double distance, double power) {
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
        driver.setStopAndResetMode();
        driver.driveDifferential(power, power);
    }

    protected void linearSlideDriveForward(int distance, double power) {
        this.linearSlideDrive(distance, power);
    }

    protected void linearSlideDriveReverse(int distance, double power) {
        this.linearSlideDrive(-distance, -power);
    }

    //  Reverse:  this.drive(-distance, -power);
    protected void linearSlideDrive(double distanceInInches, double power) {

        SimpleMotor motor = this.rosie.getLinearSlideMotor();
        motor.setTargetPosition(distanceInInches);
        motor.drive(power);
        while (motor.motorsBusy()) {
            int cp = motor.getCurrentPosition();
            telemetry.addData("ticks", cp + ", isBusy=" + motor.motorsBusy());
            telemetry.update();
        }
        motor.stop();
    }


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
    }

    protected void spin(double power) {
        Driver driver = this.rosie.getDriver();
        driver.spin(power);
    }

    protected void strafe(double power) {
        //  positive power strafe left; negative strafe right

        Driver driver = this.rosie.getDriver();
        driver.setStopAndResetMode();
        driver.strafeDifferential(power, power);
    }

    protected void turn(double degrees, double power) {

        Driver driver = this.rosie.getDriver();
        driver.setStopAndResetMode();
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
    }

    //  Don't use.
    private void gyroDrive(double power) {

        Driver driver = this.rosie.getDriver();
        MotorControllerEx motor = this.rosie.getMotorPID();
        IMUController imu = this.rosie.getIMUController();

        //  index 0:  leftPowerCorrection
        //  index 1:  rightPowerCorrection
        //  index 2:  correction value ;
        double[] p = motor.calculateDriveCorrection(power, imu.getAngle());
        driver.driveDifferential(this.normalizePower(p[0]), this.normalizePower(p[1]));
    }

    //  Don't use.
    //  positive power strafe left; negative strafe right
    private void gyroStrafe(double power) {
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

