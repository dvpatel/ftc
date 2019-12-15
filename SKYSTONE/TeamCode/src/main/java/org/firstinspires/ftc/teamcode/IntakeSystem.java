package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class IntakeSystem {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    public IntakeSystem(HardwareMap hardwareMap) {
        this.leftMotor = hardwareMap.dcMotor.get(Constants.INTAKE_LEFT_MOTOR);
        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // this.motor.setDirection(DcMotor.Direction.REVERSE);

        this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftMotor.setPower(0);

        this.rightMotor = hardwareMap.dcMotor.get(Constants.INTAKE_RIGHT_MOTOR);
        this.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // thrightMotorsetDirection(DcMotor.Direction.RrightMotor
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightMotor.setPower(0);
    }

    public void start(double percentage) {
        double power = Constants.INTAKE_MAX_POWER * Range.clip(percentage, 0, Constants.INTAKE_MAX_POWER);

        this.leftMotor.setPower(power);
        this.rightMotor.setPower(power);
    }

    public void stop() {
        this.leftMotor.setPower(0);
        this.rightMotor.setPower(0);
    }

}
