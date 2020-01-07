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

    protected void stopDriving() {
        this.rosie.getDriver().stop();
    }

    protected void driveForward(double distance, double velocity) {
        this.drive(distance, velocity);
    }

    protected void driveReverse(double distance, double velocity) {
        //  Both velocity and distance needs to be negative to go reverse
        this.drive(-distance, -velocity);
    }

    protected void strafeLeft(double distance, double velocity) {
        this.strafe(-distance, -velocity);
    }

    protected void strafeRight(double distance, double velocity) {
        //  Note:  both values must be negative ;
        this.strafe(distance, velocity);
    }

    protected void turnRight(int degrees, double velocity) {
        this.turn(degrees, velocity);
    }

    protected void turnLeft(int degrees, double velocity) {
        //  note degrees direction and velocity
        this.turn(-degrees, velocity);
    }

    protected void drive(double velocity) {
        Driver driver = this.rosie.getDriver();
        driver.setStopAndResetMode();
        driver.driveDifferential(velocity, velocity);
    }

    protected void linearSlideDriveForward(int distance, double velocity) {
        this.linearSlideDrive(distance, velocity);
    }

    protected void linearSlideDriveReverse(int distance, double velocity) {
        this.linearSlideDrive(-distance, -velocity);
    }

    //  Reverse:  this.drive(-distance, -velocity);
    protected void linearSlideDrive(double distanceInInches, double velocity) {

        SimpleMotor motor = this.rosie.getLinearSlideMotor();
        motor.setTargetPosition(distanceInInches);
        motor.drive(velocity);
        while (motor.motorsBusy()) {
            int cp = motor.getCurrentPosition();
            telemetry.addData("ticks", cp + ", isBusy=" + motor.motorsBusy());
            telemetry.update();
        }
        motor.stop();
    }


    //  PIDF optimized for driving, NOT strafing.  Strafing stalls.
    //  Reverse:  this.drive(-distance, -velocity);
    protected void drive(double distanceInInches, double velocity) {
        Driver driver = this.rosie.getDriver();
        driver.setTargetPosition(distanceInInches);
        driver.drive(velocity);
        while (opModeIsActive() && driver.motorsBusy()) {
            //  leftFront, rightFront, leftBack, rightBack;
            //  int[] cp = driver.getCurrentPosition();
            //  telemetry.addData("ticks", cp[0] + ", " + cp[1] + ", " + cp[2] + ", " + cp[3] + ", isBusy=" + driver.motorsBusy());
            telemetry.addData("Drive:  ", "Running");
            telemetry.update();
        }
        driver.stop();
    }

    //  PIDF optimized for driving, NOT strafing.  Strafing stalls.
    protected void strafe(double distanceInInches, double velocity) {

        this.rosie.getIMUController().resetAngle();

        Driver driver = this.rosie.getDriver();
        int ticks = driver.calculateTicks(distanceInInches);
        driver.setStopAndResetMode();
        driver.strafeDifferential(velocity, velocity);
        while (!driver.distanceReached(ticks)) {
            //  int[] cp = driver.getCurrentPosition();
            //  telemetry.addData("ticks", cp[0] + ", " + cp[1] + ", " + cp[2] + ", " + cp[3] + ", isBusy=" + driver.motorsBusy());
            telemetry.addData("Strafe: ", "Running");
            telemetry.update();
        }

        driver.stop();
    }


    protected void spin(double velocity) {
        Driver driver = this.rosie.getDriver();
        driver.spin(velocity);
    }

    protected void strafe(double velocity) {
        //  positive velocity strafe left; negative strafe right

        Driver driver = this.rosie.getDriver();
        driver.setStopAndResetMode();
        driver.strafeDifferential(velocity, velocity);
    }

    private double velocityToPower(double velocity) {
        return velocity/Constants.MOTOR_MAX_VELOCITY;
    }

    protected void turn(double degrees, double velocity) {

        double power = velocityToPower(velocity);
        Driver driver = this.rosie.getDriver();
        MotorControllerEx motor = this.rosie.getMotorPID();
        IMUController imu = this.rosie.getIMUController();
        imu.resetAngle();

        while (!motor.angleOnTarget()) {
            //  NOTE:  imu.getAngle returns positive on left turn, negative on right turn
            //  Negative degrees means left turn
            telemetry.addData("Degrees", -degrees);
            telemetry.addData("Angle", imu.getAngle());
            telemetry.addData("Power", power);

            double p = motor.calculateRotateCorrection(-degrees, imu.getAngle(), power);
            telemetry.addData("Correction", p);

            driver.powerDifferential(-p, p, -p, p);
            telemetry.addData("IMU Angle", imu.getAngle());
            telemetry.update();
        }

        driver.stop();
    }

    //  Don't use.
    private void gyroDrive(double velocity) {

        Driver driver = this.rosie.getDriver();
        MotorControllerEx motor = this.rosie.getMotorPID();
        IMUController imu = this.rosie.getIMUController();

        //  index 0:  leftVelocityCorrection
        //  index 1:  rightVelocityCorrection
        //  index 2:  correction value ;
        double[] p = motor.calculateDriveCorrection(velocity, imu.getAngle());
        driver.driveDifferential(p[0], p[1]);
    }

    //  Don't use.
    //  positive velocity strafe left; negative strafe right
    private void gyroStrafe(double velocity) {
        Driver driver = this.rosie.getDriver();
        MotorControllerEx motor = this.rosie.getMotorPID();
        IMUController imu = this.rosie.getIMUController();

        //  index 0:  leftPowerCorrection
        //  index 1:  rightPowerCorrection
        //  index 2:  correction value ;
        double[] p = motor.calculateDriveCorrection(velocity, imu.getAngle());
        boolean showDebug = p[0] != 0 || p[1] != 0;

        driver.strafeDifferential(p[0], p[1]);

        if (showDebug) {
            telemetry.addData("Non-Zero Angle.  Adjust velocity.", imu.getAngle());
            telemetry.update();
        }
    }
}

