package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class FoundationSystem {

    private static final int MAX_POS_DEGREE = 180;
    private static final int MIN_POS_DEGREE = 0;

    private ServoController foundationLeftServo;
    private ServoController foundationRightServo;

    public FoundationSystem(HardwareMap hardwareMap) {
        this.foundationLeftServo = new ServoController(hardwareMap, Constants.FOUNDATION_LEFT_SERVO);
        this.foundationRightServo = new ServoController(hardwareMap, Constants.FOUNDATION_RIGHT_SERVO);

        this.setPositionByDegrees(0);
    }

    private double calculatePosition(double degrees) {
        return Range.clip(degrees, 0, 180) / MAX_POS_DEGREE;
    }

    public void setPositionByDegrees(double degrees) {

        //  Offset by 10 degrees;  or fix left servo so that properly positioned;

        this.foundationLeftServo.setPosition(this.calculatePosition(degrees + 10));
        this.foundationRightServo.setPosition(this.calculatePosition(degrees));
    }

    public void triggerDown(boolean input) {

        if (input) {
            this.setPositionByDegrees(90);
        }

    }

    public void triggerUp(boolean input) {

        if (input) {
            this.setPositionByDegrees(MIN_POS_DEGREE);
        }

    }

    public double[] triggerPosition(float leftTrigger, float rightTrigger) {
        if (rightTrigger > 0.25) {
            this.triggerDown(true);
        } else if (leftTrigger > 0.25) {
            this.triggerUp(true);
        }

        double[] r = {this.foundationLeftServo.getPosition(), this.foundationRightServo.getPosition()};
        return r;
    }
}