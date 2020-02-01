package org.blueprint.ftc.core;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public abstract class AbstractLinearOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

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

    /**
     * Driving using RUN_USING_ENCODER
     * @param velocity
     */
    protected void drive(double velocity) {
        Driver driver = this.rosie.getDriver();
        driver.setStopAndResetMode();
        driver.setRunWithEncoderMode();
        driver.setDriveVelocityPID();
        driver.driveDifferential(velocity, velocity);
    }

    /**
     * Strafing
     * @param velocity
     */
    protected void strafe(double velocity) {
        //  positive velocity strafe left; negative strafe right
        Driver driver = this.rosie.getDriver();
        driver.setStopAndResetMode();
        driver.setRunWithEncoderMode();
        driver.setDriveVelocityPID();
        driver.strafeDifferential(velocity, velocity);
    }

    /**
     * Helper for drive forward with optional RUN_TO_POSITION MODE;
     * @param distance
     * @param usePositionEncoder
     */
    protected void driveForward(double distance, int speed, boolean usePositionEncoder) {
        if (usePositionEncoder) {
            this.driveToTarget(distance);
        } else {
            this.driveToTargetWithoutEncoder(distance, speed);
        }
    }

    /**
     * Helper for drive forward using RUN_TO_POSITION MODE;
     * @param distance
     */
    protected void driveForward(double distance) {
        this.driveToTarget(distance);
    }

    /**
     * Helper for reverse driving with optional RUN_TO_POSITION MODE;
     * @param distance
     * @param usePositionEncoder
     */
    protected void driveReverse(double distance, int speed, boolean usePositionEncoder) {
        if (usePositionEncoder) {
            this.driveToTarget(-distance);
        } else {
            this.driveToTargetWithoutEncoder(-distance, -speed);
        }
    }

    /**
     * Helper for reverse driving using RUN_TO_POSITION MODE;
     * @param distance
     */
    protected void driveReverse(double distance) {
        //  Both velocity and distance needs to be negative to go reverse
        this.driveToTarget(-distance);
    }

    /**
     * Helper for strafe left using RUN_WITHOUT_ENCODER MODE;
     * @param distance
     * @param velocity
     */
    protected void strafeLeft(double distance, double velocity) {
        this.strafeToTarget(-distance, -velocity);
    }

    /**
     * Helper for strafe right using RUN_WITHOUT_ENCODER MODE;
     * @param distance
     * @param velocity
     */
    protected void strafeRight(double distance, double velocity) {
        //  Note:  both values must be negative ;
        this.strafeToTarget(distance, velocity);
    }

    /**
     * Helper for turning right
     * @param degrees
     * @param velocity
     */
    protected void turnRight(int degrees, double velocity) {
        this.turn(degrees, velocity);
    }

    /**
     * Helper for turning left
     * @param degrees
     * @param velocity
     */
    protected void turnLeft(int degrees, double velocity) {
        //  note degrees direction and velocity
        this.turn(-degrees, velocity);
    }

    /**
     * Driving using RUN_TO_POSITION mode
     * @param distanceInInches
     */
    protected void driveToTarget(double distanceInInches) {
        Driver driver = this.rosie.getDriver();
        driver.setTargetPosition(distanceInInches);

        //  When using Run-to-position, set max velocity;
        //  driver.drive(velocity);
        driver.drive(Constants.MOTOR_MAX_VELOCITY);
        while (opModeIsActive() && driver.motorsBusy()) {
            telemetry.addData("Drive:  ", "Running");
            telemetry.update();
        }
        driver.stop();
    }

    protected void driveToTargetWithoutEncoder(double distanceInInches, int speed) {
        Driver driver = this.rosie.getDriver();
        int ticks = (int)(driver.calculateTicks(distanceInInches));

        this.drive(speed);
        while (opModeIsActive() && !driver.distanceReached(ticks)) {
            telemetry.addData("Driving: ", "Running");
            telemetry.update();
        }
        driver.stop();
    }

    /**
     * Strafe using RUN_WITHOUT_ENCODER
     * @param distanceInInches
     * @param velocity
     */
    protected void strafeToTarget(double distanceInInches, double velocity) {
        Driver driver = this.rosie.getDriver();

        int ticks = (int)(driver.calculateTicks(distanceInInches) * Constants.STRAFE_DISTANCE_FACTOR);

        driver.setStopAndResetMode();
        driver.strafeDifferential(velocity, velocity);
        while (opModeIsActive() && !driver.distanceReached(ticks)) {
            telemetry.addData("Strafe: ", "Running");
            telemetry.update();
        }
        driver.stop();
    }

    /**
     * Utility to convert velocity to power;
     * @param velocity
     * @return
     */
    private double velocityToPower(double velocity) {
        return velocity/Constants.MOTOR_MAX_VELOCITY;
    }

    /**
     * Turning using own PID, RUN_WITHOUT_ENCODER
     * @param degrees
     * @param velocity
     */
    protected void turn(double degrees, double velocity) {

        double power = velocityToPower(velocity);
        Driver driver = this.rosie.getDriver();
        MotorControllerEx motor = this.rosie.getMotorPID();
        IMUController imu = this.rosie.getIMUController();
        imu.resetAngle();

        //  Using external PID;  Run motors with encoders;
        driver.setStopAndResetMode();
        driver.setRunWithEncoderOffMode();

        runtime.reset();
        while (opModeIsActive() && (!motor.angleOnTarget()) && runtime.seconds() < 5) {

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
    }

}

