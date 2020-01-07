package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSystem {

    private GameBot rosie;

    private SimpleMotor linearSlideMotor;
    private ServoController linearSlideServo;
    private ServoController linearArmServo;

    private static final int DISTANCE_IN_INCHES = 24;
    private static final double POWER_LEVEL = 0.90;

    private int currentPosition;

    public LiftSystem(HardwareMap hardwareMap) {

        //  motor to control linear slide system
        this.linearSlideMotor = new SimpleMotor(hardwareMap, Constants.LINEAR_SLIDE_MOTOR_NAME);

        //  Servo attached to linear slide system;
        this.linearSlideServo = new ServoController(hardwareMap, Constants.LINEAR_SLIDE_SERVO);

        //  Servo attached to linear arm system;
        this.linearArmServo = new ServoController(hardwareMap, Constants.LINEAR_ARM_SERVO);

        //  Reset to base
        //  this.backToBase();
    }

    public SimpleMotor getLinearSlideMotor() {
        return this.linearSlideMotor;
    }

    public ServoController getLinearSlideServo() {
        return this.linearSlideServo;
    }

    public ServoController getLinearArmServo() {
        return this.linearArmServo;
    }

    private boolean armState = false;

    //  Used in tele with Gamepad;
    public void autoMode(Gamepad gamepad) {

        //  Go up / down;
        this.linearSlideMotor.drive(-gamepad.left_stick_y);

        if (gamepad.x) {
            this.linearSlideServo.setPosition(0.3);
        }

        if (gamepad.b) {
            this.linearSlideServo.setPosition(1.0);
        }


        if (gamepad.dpad_left) {
            //  open
            this.linearArmServo.setPosition(1.0);
        }

        if (gamepad.dpad_right){
            this.linearArmServo.setPosition(0);
        }
    }


    private void lift(int distanceInInches) {

        linearSlideMotor.setTargetPosition(linearSlideMotor.getCurrentPosition() + distanceInInches);
        linearSlideMotor.drive(POWER_LEVEL);
        while (linearSlideMotor.motorsBusy()) {
            this.currentPosition = linearSlideMotor.getCurrentPosition();
        }
        this.currentPosition = linearSlideMotor.getCurrentPosition();
        linearSlideMotor.stop();

        //  Disable encoders ;
        //  motor.turnOffEncoders();
    }


    public void backToBase() {
        this.releaseObject();
        this.linearSlideServo.setPositionByDegrees(0);
        this.lift(-this.linearSlideMotor.getCurrentPosition());
    }

    public void pickup(int distancesInInches) {
        this.grabObject();
        this.lift(distancesInInches);
        this.linearSlideServo.setPositionByDegrees(180);
    }

    public void drop() {
        this.releaseObject();
        this.linearSlideServo.setPositionByDegrees(0);
        this.lift(-this.linearSlideMotor.getCurrentPosition());
    }


    public void grabObject() {
        this.linearArmServo.setPositionByDegrees(180);
    }

    public void releaseObject() {
        this.linearArmServo.setPositionByDegrees(0);
    }

    private void reset() {
        this.linearSlideMotor.stop();
        this.linearSlideServo.setPositionByDegrees(0);
    }


}
