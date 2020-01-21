package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//  MG995 Servo;  Values:  0 to 1;  0 --> 0 degrees; 1 --> 180 degrees ;
//  https://www.towerpro.com.tw/product/mg995/
public class ServoController {

    private Servo servo;

    //  0.88
    private static final int MAX_POS_DEGREE = 180;
    private static final int MIN_POS_DEGREE = 0;

    public ServoController(HardwareMap hardwareMap, String deviceName) {
        this.servo = hardwareMap.get(Servo.class, deviceName);
    }

    private double calculatePosition(double degrees) {
        return Range.clip(degrees, 0, 180) / ServoController.MAX_POS_DEGREE;
    }

    public void setPositionByDegrees(double degrees) {
        this.servo.setPosition(this.calculatePosition(degrees));
    }

    //  Logic for arm servo ; lift and drop arm
    public double triggerPosition(float leftTrigger, float rightTrigger) {

        if (rightTrigger > 0.25) {
            //  Drop on stop at 10 degrees ;
            this.setPositionByDegrees(MAX_POS_DEGREE);
            rightTrigger = 0;
        } else if (leftTrigger > 0.25) {
            this.setPositionByDegrees(MIN_POS_DEGREE);
            leftTrigger = 0;
        }

        return this.servo.getPosition();

    }

    //  open, close gripper
    public double linearSlideArmTriggerPosition(boolean leftTrigger, boolean rightTrigger) {

        if (leftTrigger) {
            //  open;
            this.openGripper();
        } else if (rightTrigger) {
            this.closeGripper();
        }

        return this.servo.getPosition();
    }

    public void openGripper() {
        this.setPositionByDegrees(0);
    }

    public void closeGripper() {
        this.setPositionByDegrees(90);
    }

    public void setPosition(double inp) {
        this.servo.setPosition(Range.clip(inp, 0, 1.0));
    }

    public double getPosition() {
        return this.servo.getPosition();
    }

    public Servo.Direction getDir() {
        return this.servo.getDirection();
    }

}
