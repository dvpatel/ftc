package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.blueprint.ftc.core.Constants;

/**
 * Robot driver
 */
public class Driver {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;

    public Driver(HardwareMap hardwareMap) {
        this(hardwareMap, Constants.LEFT_FRONT_MOTOR_NAME, Constants.RIGHT_FRONT_MOTOR_NAME, Constants.LEFT_BACK_MOTOR_NAME, Constants.RIGHT_BACK_MOTOR_NAME);
    }

    public Driver(HardwareMap hardwareMap, String leftFrontDeviceName, String rightFrontDeviceName, String leftBackDeviceName, String rightBackDeviceName) {
        this.setLeftMotor(hardwareMap, leftFrontDeviceName, leftBackDeviceName);
        this.setRightMotor(hardwareMap, rightFrontDeviceName, rightBackDeviceName);

        //  this.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setRunWithoutEncoderMode();
        this.driveDifferential(0, 0);
    }

    private void setRightMotor(HardwareMap hardwareMap, String rightFrontDeviceName, String rightBackDeviceName) {
        this.rightFrontMotor = hardwareMap.dcMotor.get(rightFrontDeviceName);
        this.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        this.rightBackMotor = hardwareMap.dcMotor.get(rightBackDeviceName);
        this.rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    private void setLeftMotor(HardwareMap hardwareMap, String leftFrontDeviceName, String leftBackDeviceName) {
        this.leftFrontMotor = hardwareMap.dcMotor.get(leftFrontDeviceName);
        this.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.leftBackMotor = hardwareMap.dcMotor.get(leftBackDeviceName);
        this.leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //  Calculate ticks for given inches ;
    public int calculateTicks(double distanceInInches) {
        return (int) (distanceInInches * Constants.TICK_DIAMETER_RATIO);
    }

    public void setTargetPosition(double distanceInInches) {

        //  Always reset;  starts at zero;
        this.setStopAndResetMode();

        //  Run based on speed, not power;  OR run to target using position and power;
        //  this.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  Tells motor to run to target using position and power ;  Make sure t reset encoder when done!
        this.setRunWithoutEncoderMode();

        //  set target
        this.setTicksToTargets(distanceInInches);

        this.setRunToPositionMode();

        //  Apply power, somewhere ;  MAKE sure to turn off encoder when done.
    }

    //  Revisit;  Are all encoders needed?  Also should only one motor be used for calc?
    public boolean motorsBusy() {
        return this.leftFrontMotor.isBusy() || this.rightFrontMotor.isBusy();
    }

    public void turnOffEncoders() {
        this.setStopAndResetMode();
        this.setRunWithoutEncoderMode();
    }

    private void setStopAndResetMode() {
        this.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunWithoutEncoderMode() {
        this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setRunToPositionMode() {
        this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setTicksToTargets(double distanceInInches) {
        int ticks = this.calculateTicks(distanceInInches);
        this.leftFrontMotor.setTargetPosition(ticks);
        this.rightFrontMotor.setTargetPosition(ticks);
    }

    public void powerDifferential(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        this.leftBackMotor.setPower(leftBackPower);
        this.rightBackMotor.setPower(rightBackPower);
        this.leftFrontMotor.setPower(leftFrontPower);
        this.rightFrontMotor.setPower(rightFrontPower);
    }

    //  Current position in ticks;
    public int[] getCurrentPosition() {
        int[] results = {
                this.leftFrontMotor.getCurrentPosition(),
                this.rightFrontMotor.getCurrentPosition()
        };
        return results;
    }

    public double[] getPowerForEncodedMotors() {
        double[] results = {
                this.leftFrontMotor.getController().getMotorPower(this.leftFrontMotor.getPortNumber()),
                this.rightFrontMotor.getController().getMotorPower(this.rightFrontMotor.getPortNumber())
        };

        return results;
    }

    //  Drive forward or backward
    public void drive(double power) {
        this.driveDifferential(power, power);
    }

    public void driveDifferential(double leftPower, double rightPower) {
        this.powerDifferential(leftPower, rightPower, leftPower, rightPower);
    }

    //  Default:  positive power strafe right
    public void strafe(double power) {
        this.strafeDifferential(power, power);
    }

    //  Default:  positive power strafes right
    public void strafeDifferential(double p1, double p2) {
        this.powerDifferential(p1, -p2, -p2, p1);
    }

    public void spin(double power) {
        //  LF; RF; LB; RB
        this.powerDifferential(power, -power, power, -power);
    }

    //  Wide turn?
    private void wideTurn(double power) {
        throw new RuntimeException("Not implemented.  Really needed?");
    }

    //  positive power:  left forwrae diagnol; negative power:  reverse right diagnoal
    public void leftDiagonalStrafe(double power) {
        this.powerDifferential(0, power, power, 0);
    }

    //  power power:  right forward diagnol; negative power:  reverse left diagnoal
    public void rightDiagonalStrafe(double power) {
        this.powerDifferential(power, 0, 0, power);
    }

    public void stop() {
        this.powerDifferential(0, 0, 0, 0);
    }

}