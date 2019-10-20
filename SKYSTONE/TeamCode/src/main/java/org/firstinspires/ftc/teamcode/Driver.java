package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;

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

    private void setLeftMotor(HardwareMap hardwareMap, String leftFrontDeviceName, String leftBackDeviceName) {
        this.leftFrontMotor = hardwareMap.dcMotor.get(leftFrontDeviceName);
        this.leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        this.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.leftBackMotor = hardwareMap.dcMotor.get(leftBackDeviceName);
        this.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        this.leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setRightMotor(HardwareMap hardwareMap, String rightFrontDeviceName, String rightBackDeviceName) {
        this.rightFrontMotor = hardwareMap.dcMotor.get(rightFrontDeviceName);
        this.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.rightBackMotor = hardwareMap.dcMotor.get(rightBackDeviceName);
        this.rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //  Calculate ticks for given inches ;
    public int calculateTicks(double distanceInInches) {
        return (int) (distanceInInches * Constants.TICK_DIAMETER_RATIO);
    }

    //  Sample:  Drive to Target in inches ;
    private void driveToTargetExample(double distanceInInches, double leftPower, double rightPower) {

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

        this.leftFrontMotor.setTargetPosition(ticks);
        this.leftBackMotor.setTargetPosition(ticks);
        this.rightFrontMotor.setTargetPosition(ticks);
        this.rightBackMotor.setTargetPosition(ticks);

        //  Tells motor to run to target ;
        this.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        this.leftBackMotor.setMode(mode);
        this.rightFrontMotor.setMode(mode);
        this.rightBackMotor.setMode(mode);

    }

    //  Drive forward or backward
    public void drive(double power) {
        this.driveDifferential(power, power);
    }

    public void driveDifferential(double leftPower, double rightPower) {
        this.leftFrontMotor.setPower(leftPower);
        this.leftBackMotor.setPower(leftPower);

        this.rightFrontMotor.setPower(rightPower);
        this.rightBackMotor.setPower(rightPower);
    }


    //  Default:  positive power strafe left
    public void strafe(double power) {
        this.strafeDifferential(power, power);
    }

    //  Default:  positive power strafes left
    public void strafeDifferential(double p1, double p2) {
        this.leftFrontMotor.setPower(-p1);
        this.rightFrontMotor.setPower(p2);

        this.leftBackMotor.setPower(p2);
        this.rightBackMotor.setPower(-p1);
    }

    //  positive power rotates left ;
    public void rotate(double power) {
        this.leftFrontMotor.setPower(-power);
        this.rightFrontMotor.setPower(power);

        this.leftBackMotor.setPower(-power);
        this.rightBackMotor.setPower(power);
    }


    private void diagonalStrafe(double p1, double p2) {
        this.leftFrontMotor.setPower(p1);
        this.rightFrontMotor.setPower(p2);

        this.leftBackMotor.setPower(p2);
        this.rightBackMotor.setPower(p1);
    }

    public void leftDiagonalStrafe(double power) {
        this.diagonalStrafe(0, power);
    }

    public void rightDiagonalStrafe(double power) {
        this.diagonalStrafe(power, 0);
    }


    public void stop() {
        this.driveDifferential(0, 0);
    }

}