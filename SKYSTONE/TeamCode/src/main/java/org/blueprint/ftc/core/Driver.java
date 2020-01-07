package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.blueprint.ftc.core.Constants;

/**
 * Robot driver;  Always uses encoder mode.  PIDF must be reset if weight changes;  See MaxVelocityTest
 * RUN_USING_ENCODER:  Velocity in ticks per second of motor;  use setVelocity;  Need to determine with Load;
 */
public class Driver {

    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;

    public Driver(HardwareMap hardwareMap) {
        this(hardwareMap, Constants.LEFT_FRONT_MOTOR_NAME, Constants.RIGHT_FRONT_MOTOR_NAME, Constants.LEFT_BACK_MOTOR_NAME, Constants.RIGHT_BACK_MOTOR_NAME);
    }

    public Driver(HardwareMap hardwareMap, String leftFrontDeviceName, String rightFrontDeviceName, String leftBackDeviceName, String rightBackDeviceName) {
        this.setLeftMotor(hardwareMap, leftFrontDeviceName, leftBackDeviceName);
        this.setRightMotor(hardwareMap, rightFrontDeviceName, rightBackDeviceName);

        this.setStopAndResetMode();
        this.setRunWithEncoderMode();
        this.stop();
    }

    private void setRightMotor(HardwareMap hardwareMap, String rightFrontDeviceName, String rightBackDeviceName) {
        this.rightFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get(rightFrontDeviceName);
        this.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        this.rightBackMotor = (DcMotorEx) hardwareMap.dcMotor.get(rightBackDeviceName);
        this.rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    private void setLeftMotor(HardwareMap hardwareMap, String leftFrontDeviceName, String leftBackDeviceName) {
        this.leftFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get(leftFrontDeviceName);
        this.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.leftBackMotor = (DcMotorEx) hardwareMap.dcMotor.get(leftBackDeviceName);
        this.leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //  Must set properly;
    private void setVelocityPID() {
        //  Set for each motor?

        //  Get values from MaxVelocityTest;
        this.leftFrontMotor.setVelocityPIDFCoefficients(Constants.PID_DRIVE_KP, Constants.PID_DRIVE_KI, Constants.PID_DRIVE_KD, Constants.PID_DRIVE_KF);
        this.rightFrontMotor.setVelocityPIDFCoefficients(Constants.PID_DRIVE_KP, Constants.PID_DRIVE_KI, Constants.PID_DRIVE_KD, Constants.PID_DRIVE_KF);
        this.leftBackMotor.setVelocityPIDFCoefficients(Constants.PID_DRIVE_KP, Constants.PID_DRIVE_KI, Constants.PID_DRIVE_KD, Constants.PID_DRIVE_KF);
        this.rightBackMotor.setVelocityPIDFCoefficients(Constants.PID_DRIVE_KP, Constants.PID_DRIVE_KI, Constants.PID_DRIVE_KD, Constants.PID_DRIVE_KF);

        this.leftFrontMotor.setPositionPIDFCoefficients(5.0);
        this.rightFrontMotor.setPositionPIDFCoefficients(5.0);
        this.leftBackMotor.setPositionPIDFCoefficients(5.0);
        this.rightBackMotor.setPositionPIDFCoefficients(5.0);
    }

    //  Calculate ticks for given inches ;
    public int calculateTicks(double distanceInInches) {
        return (int) (distanceInInches * Constants.TICK_GEAR_RATIO) ;
    }

    public void setTargetPosition(double distanceInInches) {

        //  run w/o encoders
        //  Always reset;  starts at zero;
        this.setStopAndResetMode();
        this.setRunWithEncoderMode();

        //  set target
        this.setTicksToTargets(distanceInInches);

        //  Run to target position;
        this.setRunToPositionMode();

        //  Apply velocity, somewhere ;  MAKE sure to turn off encoder when done.
    }

    public void setStrafeTargetPosition(double distanceInInches) {

        //  run w/o encoders
        //  Always reset;  starts at zero;
        this.setStopAndResetMode();
        this.setRunWithEncoderOffMode();

        //  set target
        //  this.setTicksToTargetsForStrafe(distanceInInches);

        //  Run to target position;
        //  this.setRunToPositionMode();

        //  Apply velocity, somewhere ;  MAKE sure to turn off encoder when done.
    }

    //  Revisit;  Are all encoders needed?  Also should only one motor be used for calc?
    public boolean motorsBusy() {
        return this.leftFrontMotor.isBusy() &&
                this.rightFrontMotor.isBusy() &&
                this.leftBackMotor.isBusy() &&
                this.rightBackMotor.isBusy();
    }

    public void setStopAndResetMode() {
        this.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunWithEncoderMode() {
        this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.setVelocityPID();
    }

    private void setRunWithEncoderOffMode() {
        this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setRunToPositionMode() {
        this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTicksToTargets(double distanceInInches) {
        int ticks = this.calculateTicks(distanceInInches);

        this.leftFrontMotor.setTargetPosition(ticks);
        this.rightFrontMotor.setTargetPosition(ticks);
        this.leftBackMotor.setTargetPosition(ticks);
        this.rightBackMotor.setTargetPosition(ticks);
    }

    public void setTicksToTargetsForStrafe(double distanceInInches) {
        int ticks = this.calculateTicks(distanceInInches);
        this.leftFrontMotor.setTargetPosition(ticks);
        this.rightFrontMotor.setTargetPosition(-ticks);
        this.leftBackMotor.setTargetPosition(-ticks);
        this.rightBackMotor.setTargetPosition(ticks);
    }

    private double normalizeVelocity(double velocity) {
        return Range.clip(velocity, -Constants.MOTOR_MAX_VELOCITY, Constants.MOTOR_MAX_VELOCITY);
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

    //  Drive forward or backward
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

    public void spin(double velocity) {
        //  LF; RF; LB; RB
        this.velocityDifferential(velocity, -velocity, velocity, -velocity);
    }

    //  Wide turn?
    private void wideTurn(double velocity) {
        throw new RuntimeException("Not implemented.  Really needed?");
    }

    //  positive velocity:  left forwrae diagnol; negative velocity:  reverse right diagnoal
    public void leftDiagonalStrafe(double velocity) {
        this.velocityDifferential(0, velocity, velocity, 0);
    }

    //  velocity:  right forward diagnol; negative velocity:  reverse left diagnoal
    public void rightDiagonalStrafe(double velocity) {
        this.velocityDifferential(velocity, 0, 0, velocity);
    }

    public void stop() {
        this.velocityDifferential(0, 0, 0, 0);
    }

}