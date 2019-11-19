package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

        this.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    //  Sample:  Drive to Target in inches ;
    public void driveToTargetExample(double distanceInInches, double leftPower, double rightPower) {

        this.setTargetPosition(distanceInInches);

        //  Now drive ;
        this.driveDifferential(leftPower, rightPower);

        //  while busy, do something ;
        while (this.motorsBusy()) {
            //  Something something...
            System.out.println("Do nothing while driving..");
        }

        // Done driving; now stop
        //  stop after done.
        this.stop();
    }

    public void setTargetPosition(double distanceInInches) {
        int ticks = this.calculateTicks(distanceInInches);

        //  Always reset;  starts at zero;
        this.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  set target
        this.setTicksToTargets(ticks);

        //  Tells motor to run to target ;
        this.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        //  Apply power, somewhere ;
    }

    //  Revisit;  Are all encoders needed?
    public boolean motorsBusy() {
        return this.leftFrontMotor.isBusy() &&
                this.rightFrontMotor.isBusy() &&
                this.leftBackMotor.isBusy() &&
                this.rightBackMotor.isBusy();
    }


    private void setDriveMode(DcMotor.RunMode mode) {
        this.leftFrontMotor.setMode(mode);
        this.rightFrontMotor.setMode(mode);

        //  this.leftBackMotor.setMode(mode);
        //  this.rightBackMotor.setMode(mode);
    }

    private void setTicksToTargets(int ticks) {
        this.leftFrontMotor.setTargetPosition(ticks);
        this.rightFrontMotor.setTargetPosition(ticks);

        //  this.leftBackMotor.setTargetPosition(ticks);
        //  this.rightBackMotor.setTargetPosition(ticks);
    }

    public void powerDifferential(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        this.leftFrontMotor.setPower(leftFrontPower);
        this.rightFrontMotor.setPower(rightFrontPower);
        this.leftBackMotor.setPower(leftBackPower);
        this.rightBackMotor.setPower(rightBackPower);
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

    public void rotate(double power) {
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