package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;

public class IntakeSystem {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private CRServoController leftServo;
    private CRServoController rightServo;
    private boolean servosDown;


    private boolean isOn;  // on, off switch
    private double intakePower;

    public IntakeSystem(HardwareMap hardwareMap) {
        this.initDCMotors(hardwareMap);
        this.initCRServos(hardwareMap);
    }

    private void initDCMotors(HardwareMap hardwareMap) {
        this.leftMotor = hardwareMap.dcMotor.get(Constants.INTAKE_LEFT_MOTOR);
        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftMotor.setDirection(DcMotor.Direction.REVERSE);
        this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.rightMotor = hardwareMap.dcMotor.get(Constants.INTAKE_RIGHT_MOTOR);
        this.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initCRServos(HardwareMap hardwareMap) {
        //  Setup short arm servo;
        this.leftServo = new CRServoController(hardwareMap, Constants.INTAKE_LEFT_SERVO, Constants.INTAKE_LEFT_SERVO_REVERSE);
        this.rightServo = new CRServoController(hardwareMap, Constants.INTAKE_RIGHT_SERVO, Constants.INTAKE_RIGHT_SERVO_REVERSE);

        //  this.setIntakeServosInitPosition();
    }

    public void setIntakeServosInitPosition() {
        if (!this.servosDown) {
            this.leftServo.setPower(Constants.INTAKE_LEFT_SERVO_INIT_POWER);
            this.rightServo.setPower(Constants.INTAKE_RIGHT_SERVO_INIT_POWER);

            this.servosDown = true;
        }
    }

    public void setIntakeServosPower(double power) {
        this.leftServo.setPower(power);
        this.rightServo.setPower(power);
    }

    public void resetIntakeServos() {
        this.setIntakeServosPower(0.0);
        this.servosDown = false;
    }

    public void setDCMotorsPower(double power) {
        this.leftMotor.setPower(power);
        this.rightMotor.setPower(power);
    }

    public void stopDCMotors() {
        this.setDCMotorsPower(0);
    }

    public void start() {
        this.setDCMotorsPower(1.0);
    }

    public void stop() {
        this.stopDCMotors();
        //  this.resetIntakeServos();
    }

    //  Game mode w/ Switch and right, left bumper
    public void autoMode(boolean leftBumper, boolean rightBumper, boolean isOff) {
        if (leftBumper) {
            this.intakePower = -1.0;
            this.isOn = true;
        } else if (rightBumper) {
            this.intakePower = 1.0;
            this.isOn = true;
        } else if (isOff) {
            this.intakePower = 0.0;
            this.isOn = false;
            this.stop();
        }

        if (this.isOn) {
            this.setDCMotorsPower(this.intakePower);
        }
    }
}
