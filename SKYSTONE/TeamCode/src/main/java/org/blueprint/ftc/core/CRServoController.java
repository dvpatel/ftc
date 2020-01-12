package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//  Class for continuous rotation servo; -1 to 1;
public class CRServoController {

    private CRServo servo;

    public CRServoController(HardwareMap hardwareMap, String deviceName, boolean isReverse) {
        this.servo = hardwareMap.get(CRServo.class, deviceName);

        if(isReverse) {
            this.servo.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void setPower(double power) {
        this.servo.setPower(power);
    }

    public double getPower() {
        return this.servo.getPower();
    }
}
