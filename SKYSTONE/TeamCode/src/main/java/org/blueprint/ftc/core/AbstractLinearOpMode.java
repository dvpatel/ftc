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

    protected void drive(double velocity) {
        Driver driver = this.rosie.getDriver();
        driver.setStopAndResetMode();
        driver.driveDifferential(velocity, velocity);
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

    //  PIDF optimized for driving, NOT strafing.  Strafing stalls.
    //  Reverse:  this.drive(-distance, -velocity);
    protected void drive(double distanceInInches, double velocity) {
        Driver driver = this.rosie.getDriver();
        driver.setTargetPosition(distanceInInches);

        //  When using Run-to-position, set max velocity;
        //  driver.drive(velocity);
        driver.drive(Constants.MOTOR_MAX_VELOCITY);

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
        Driver driver = this.rosie.getDriver();

        int ticks = (int)(driver.calculateTicks(distanceInInches) * Constants.STRAFE_DISTANCE_FACTOR);

        driver.setStopAndResetMode();
        driver.strafeDifferential(velocity, velocity);
        while (!driver.distanceReached(ticks)) {
            telemetry.addData("Strafe: ", "Running");
            telemetry.addData("Target Ticks", ticks );

            int[] cp = driver.getCurrentPosition();
            telemetry.addData("Actual Ticks", cp[0] + ", " + cp[1] + ", " + cp[2] + ", " + cp[3] );

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

        //  Using external PID;  Run motors with encoders;
        driver.setStopAndResetMode();
        driver.setRunWithEncoderOffMode();

        while (!motor.angleOnTarget()) {
            //  NOTE:  Rotation on Z-Axis, therefore
            //  imu.getAngle returns positive on left turn, negative on right turn
            //  Negative degrees means left turn
            telemetry.addData("Target Angle", -degrees);
            telemetry.addData("Current Angle", imu.getAngle());

            telemetry.addData("Current Power", power);

            double p = motor.calculateRotateCorrection(-degrees, imu.getAngle(), power);
            telemetry.addData("Power Correction", p);

            driver.powerDifferential(-p, p, -p, p);
            telemetry.update();
        }

        driver.stop();

        driver.setStopAndResetMode();
        driver.setRunWithEncoderMode();
    }

}

