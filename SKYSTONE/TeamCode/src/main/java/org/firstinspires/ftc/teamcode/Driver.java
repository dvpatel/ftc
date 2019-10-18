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

    public void driveDifferential(double leftPower, double rightPower) {
        this.leftFrontMotor.setPower(leftPower);
        this.leftBackMotor.setPower(leftPower);

        this.rightFrontMotor.setPower(rightPower);
        this.rightBackMotor.setPower(rightPower);
    }

    //  Drive forward or backward
    public void drive(double power) {
        this.leftFrontMotor.setPower(power);
        this.leftBackMotor.setPower(power);

        this.rightFrontMotor.setPower(power);
        this.rightBackMotor.setPower(power);
    }

    //  Default:  positive power strifes left
    public void strife(double power) {
        this.leftFrontMotor.setPower(-power);
        this.rightFrontMotor.setPower(power);

        this.leftBackMotor.setPower(power);
        this.rightBackMotor.setPower(-power);
    }

    //  positive power rotates left ;
    public void rotate(double power) {
        this.leftFrontMotor.setPower(-power);
        this.rightFrontMotor.setPower(power);

        this.leftBackMotor.setPower(-power);
        this.rightBackMotor.setPower(power);
    }


    private void diagonalStrife(double p1, double p2) {
        this.leftFrontMotor.setPower(p1);
        this.rightFrontMotor.setPower(p2);

        this.leftBackMotor.setPower(p2);
        this.rightBackMotor.setPower(p1);
    }

    public void leftDiagonalStrife(double power) {
        this.diagonalStrife(0, power);
    }

    public void rightDiagonalStrife(double power) {
        this.diagonalStrife(power, 0);
    }


    public void stop() {
        this.driveDifferential(0, 0);
    }

}