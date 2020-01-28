package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Robot driver;  Always uses encoder mode.  PIDF must be reset if weight changes;  See MaxVelocityTest
 * RUN_USING_ENCODER:  Velocity in ticks per second of motor;  use setVelocity;  Need to determine with Load;
 */
public final class Driver {

    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;

    /**
     * Configure with motor names in Constants file.
     * @param hardwareMap
     */
    public Driver(HardwareMap hardwareMap) {
        this(hardwareMap, Constants.LEFT_FRONT_MOTOR_NAME, Constants.RIGHT_FRONT_MOTOR_NAME, Constants.LEFT_BACK_MOTOR_NAME, Constants.RIGHT_BACK_MOTOR_NAME);
    }

    /**
     * Configure motors with specific motor names.
     * @param hardwareMap
     * @param leftFrontDeviceName
     * @param rightFrontDeviceName
     * @param leftBackDeviceName
     * @param rightBackDeviceName
     */
    public Driver(HardwareMap hardwareMap, String leftFrontDeviceName, String rightFrontDeviceName, String leftBackDeviceName, String rightBackDeviceName) {
        this.setLeftMotor(hardwareMap, leftFrontDeviceName, leftBackDeviceName);
        this.setRightMotor(hardwareMap, rightFrontDeviceName, rightBackDeviceName);

        this.setStopAndResetMode();
        this.setRunWithEncoderMode();
        this.stop();
    }

    /**
     * Configuration for left motors;
     * @param hardwareMap
     * @param leftFrontDeviceName
     * @param leftBackDeviceName
     */
    private void setLeftMotor(HardwareMap hardwareMap, String leftFrontDeviceName, String leftBackDeviceName) {
        this.leftFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get(leftFrontDeviceName);
        this.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        this.leftBackMotor = (DcMotorEx) hardwareMap.dcMotor.get(leftBackDeviceName);
        this.leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Configuration for right motors;
     * @param hardwareMap
     * @param rightFrontDeviceName
     * @param rightBackDeviceName
     */
    private void setRightMotor(HardwareMap hardwareMap, String rightFrontDeviceName, String rightBackDeviceName) {
        this.rightFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get(rightFrontDeviceName);
        this.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.rightBackMotor = (DcMotorEx) hardwareMap.dcMotor.get(rightBackDeviceName);
        this.rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * VelocityPID setup;  Required for RUN_USING_ENCODER MODE; Values determined from MaxVelocityTest
     */
    public void setDriveVelocityPID() {
        //  Get values from MaxVelocityTest;

        this.leftFrontMotor.setVelocityPIDFCoefficients(Constants.PID_DRIVE_KP, Constants.PID_DRIVE_KI, Constants.PID_DRIVE_KD, Constants.PID_DRIVE_KF);
        this.rightFrontMotor.setVelocityPIDFCoefficients(Constants.PID_DRIVE_KP, Constants.PID_DRIVE_KI, Constants.PID_DRIVE_KD, Constants.PID_DRIVE_KF);
        this.leftBackMotor.setVelocityPIDFCoefficients(Constants.PID_DRIVE_KP, Constants.PID_DRIVE_KI, Constants.PID_DRIVE_KD, Constants.PID_DRIVE_KF);
        this.rightBackMotor.setVelocityPIDFCoefficients(Constants.PID_DRIVE_KP, Constants.PID_DRIVE_KI, Constants.PID_DRIVE_KD, Constants.PID_DRIVE_KF);
    }

    /**
     *  PositionalPID setup.  Required for Run_To_Position MODE;  Values determined from MaxVelocityTest
     */
    public void setPositionalPID() {
        //  Get values from MaxVelocityTest;
        this.leftFrontMotor.setPositionPIDFCoefficients(Constants.POSITIONAL_DRIVE_KP);
        this.rightFrontMotor.setPositionPIDFCoefficients(Constants.POSITIONAL_DRIVE_KP);
        this.leftBackMotor.setPositionPIDFCoefficients(Constants.POSITIONAL_DRIVE_KP);
        this.rightBackMotor.setPositionPIDFCoefficients(Constants.POSITIONAL_DRIVE_KP);
    }

    /**
     * Driving using RUN_TO_POSITION MODE;  For driving forward and reverse;
     * @param distanceInInches
     */
    public void setTargetPosition(double distanceInInches) {

        //  run w/o encoders
        //  Always reset;  starts at zero;  Run in this order!
        this.setStopAndResetMode();
        this.setRunWithEncoderMode();
        this.setDriveVelocityPID();
        this.setTicksToTargets(distanceInInches);

        //  Apply velocity, somewhere ;  MAKE sure to turn off encoder when done.
    }

    /**
     * For strafing using RUN_USING_ENCODER
     * @param distanceInInches
     */
    public void setStrafeTargetPosition(double distanceInInches) {

        //  run using encoders
        //  Always reset;  starts at zero;  Run in this order!
        this.setStopAndResetMode();
        this.setRunWithEncoderOffMode();

        //  Apply velocity, somewhere ;  MAKE sure to turn off encoder when done.
    }

    /**
     * Motor busy test.
     * @return
     */
    public boolean motorsBusy() {
        return this.leftFrontMotor.isBusy() &&
                this.rightFrontMotor.isBusy() &&
                this.leftBackMotor.isBusy() &&
                this.rightBackMotor.isBusy();
    }

    /**
     * Rest encoder / tick values;
     */
    public void setStopAndResetMode() {
        this.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Set RUN_USING_ENCODER mode
     */
    public void setRunWithEncoderMode() {
        this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * set RUN_WITHOUT_ENCODER mode
     */
    public void setRunWithEncoderOffMode() {
        this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Set RUN_TO_POSITION mode
     */
    public void setRunToPositionMode() {
        this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Calculate ticks for given inches ;
     * @param distanceInInches
     * @return
     */
    public int calculateTicks(double distanceInInches) {
        return (int) (distanceInInches * Constants.TICK_GEAR_RATIO) ;
    }

    /**
     *  Calculate inches given ticks;
     * @param ticks
     * @return
     */
    public double toInches(double ticks) {
        return (ticks/Constants.TICK_GEAR_RATIO);
    }

    /**
     * For RUN_TO_POSITION driving;
     * @param distanceInInches
     */
    public void setTicksToTargets(double distanceInInches) {

        //  Keep this order;
        this.setPositionalPID();

        int ticks = this.calculateTicks(distanceInInches);
        this.leftFrontMotor.setTargetPosition(ticks);
        this.rightFrontMotor.setTargetPosition(ticks);
        this.leftBackMotor.setTargetPosition(ticks);
        this.rightBackMotor.setTargetPosition(ticks);

        this.setRunToPositionMode();
    }

    /**
     * For strafing using RUN_WITHOUT_ENCODER;
     * @param distanceInInches
     */
    public void setTicksToTargetsForStrafe(double distanceInInches) {
        int ticks = (int) (this.calculateTicks(distanceInInches) * Constants.STRAFE_DISTANCE_FACTOR);
        this.leftFrontMotor.setTargetPosition(ticks);
        this.rightFrontMotor.setTargetPosition(-ticks);
        this.leftBackMotor.setTargetPosition(-ticks);
        this.rightBackMotor.setTargetPosition(ticks);
    }

    public void velocityDifferential(double leftFrontVelocity, double rightFrontVelocity, double leftBackVelocity, double rightBackVelocity) {
        this.leftFrontMotor.setVelocity(leftFrontVelocity);
        this.rightFrontMotor.setVelocity(rightFrontVelocity);
        this.leftBackMotor.setVelocity(leftBackVelocity);
        this.rightBackMotor.setVelocity(rightBackVelocity);
    }

    //  Use only for right / left turn using Gyro
    public void powerDifferential(double lfPow, double rfPow, double lbPow, double rbPow) {
        this.leftFrontMotor.setPower(lfPow);
        this.rightFrontMotor.setPower(rfPow);
        this.leftBackMotor.setPower(lbPow);
        this.rightBackMotor.setPower(rbPow);
    }

    public boolean distanceReached(int ticks) {

        //  negative ticks;
        //  -5 -1200

        int absTicks = Math.abs(ticks);
        return (Math.abs(this.leftFrontMotor.getCurrentPosition()) >= absTicks ||
                Math.abs(this.rightFrontMotor.getCurrentPosition()) >= absTicks ||
                Math.abs(this.leftBackMotor.getCurrentPosition()) >= absTicks ||
                Math.abs(this.rightBackMotor.getCurrentPosition()) >= absTicks);
    }

    //  Current position in ticks;
    public int[] getCurrentPosition() {
        int[] results = {
                this.leftFrontMotor.getCurrentPosition(),
                this.rightFrontMotor.getCurrentPosition(),
                this.leftBackMotor.getCurrentPosition(),
                this.rightBackMotor.getCurrentPosition()
        };

        //  leftFront, rightFront, leftBack, rightBack
        return results;
    }

    public double[] getVelocity() {
        double[] results = {
                this.leftFrontMotor.getVelocity(),
                this.rightFrontMotor.getVelocity(),
                this.leftBackMotor.getVelocity(),
                this.rightBackMotor.getVelocity()
        };

        //  leftFront, rightFront, leftBack, rightBack
        return results;
    }

    /**
     * Drive forward or backward
     * @param velocity
     */
    public void drive(double velocity) {
        this.driveDifferential(velocity, velocity);
    }

    public void driveDifferential(double leftVelocity, double rightVelocity) {
        this.velocityDifferential(leftVelocity, rightVelocity, leftVelocity, rightVelocity);
    }

    //  Default:  positive velocity strafe right
    public void strafe(double velocity) {
        this.strafeDifferential(velocity, velocity);
    }

    //  Default:  positive velocity strafes right
    public void strafeDifferential(double p1, double p2) {
        this.velocityDifferential(p1, -p2, -p2, p1);
    }

    public void stop() {
        this.velocityDifferential(0, 0, 0, 0);
    }
}