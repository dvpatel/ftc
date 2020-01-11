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
    //  returns motor positio, arm position, slideservo position;
    public double[] autoMode(Gamepad gamepad) {

        //  If max or min height, stop;
        float yVal = -gamepad.left_stick_y;
        if (((this.linearSlideMotor.getCurrentPosition() >= Constants.SIMPLE_WHEEL_MAX_TICKS) && yVal > 0 ) ||
                ((this.linearSlideMotor.getCurrentPosition() <= 10) && yVal < 0) ) {
            yVal = 0;
        }
        this.linearSlideMotor.drive(yVal);


        if (gamepad.x) {
            this.linearSlideServo.setPosition(0.3);
        }

        if (gamepad.b) {
            this.linearSlideServo.setPosition(1.0);
        }

        //  open, close
        this.linearArmServo.linearSlideArmTriggerPosition(gamepad.dpad_left, gamepad.dpad_right);

        double[] r = { this.linearSlideMotor.getCurrentPosition(), this.linearArmServo.getPosition(), this.linearSlideServo.getPosition() };
        return r;
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
