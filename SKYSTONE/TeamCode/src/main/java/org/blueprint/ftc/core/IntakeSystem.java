package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSystem {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    public IntakeSystem(HardwareMap hardwareMap) {
        this.leftMotor = hardwareMap.dcMotor.get(Constants.INTAKE_LEFT_MOTOR);
        this.leftMotor.setDirection(DcMotor.Direction.REVERSE);
        this.rightMotor = hardwareMap.dcMotor.get(Constants.INTAKE_RIGHT_MOTOR);
    }

    public void power(double power) {

        if (power > 0) {
            this.leftMotor.setPower(1);
            this.rightMotor.setPower(1);
        } else if (power < 0) {
            this.leftMotor.setPower(-1);
            this.rightMotor.setPower(-1);
        } else {
            this.leftMotor.setPower(0);
            this.rightMotor.setPower(0);
        }

    }

    public void stop() {
        this.power(0);
    }


}
