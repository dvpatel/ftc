package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSystem {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private boolean isOn;  // on, off switch
    private double intakePower;

    public IntakeSystem(HardwareMap hardwareMap) {
        this.leftMotor = hardwareMap.dcMotor.get(Constants.INTAKE_LEFT_MOTOR);
        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftMotor.setDirection(DcMotor.Direction.REVERSE);
        this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.rightMotor = hardwareMap.dcMotor.get(Constants.INTAKE_RIGHT_MOTOR);
        this.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void power(double power) {
        this.leftMotor.setPower(power);
        this.rightMotor.setPower(power);
    }

    public void stop() {
        this.power(0);
    }

    //  Autonomous mode w/ Switch and right, left bumper
    public void autoMode(Gamepad gamepad) {
        if (gamepad.left_bumper) {
            this.intakePower = -1;
            this.isOn = true;
        }

        if (gamepad.right_bumper) {
            this.intakePower = 1;
            this.isOn = true;
        }

        if (gamepad.a) {
            this.intakePower = 0;
            this.isOn = false;
            this.stop();
        }

        if (this.isOn) {
            this.power(this.intakePower);
        }
    }
}
