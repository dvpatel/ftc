package org.blueprint.ftc.core;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class LiftSystem {

    private SimpleMotor linearSlideMotor;
    private ServoController linearSlideServo;
    private ServoController linearArmServo;

    private static final double POWER_LEVEL = 0.90;

    public LiftSystem(HardwareMap hardwareMap) {

        //  motor to control linear slide system
        this.linearSlideMotor = new SimpleMotor(hardwareMap, Constants.LINEAR_SLIDE_MOTOR_NAME);

        //  Servo attached to linear slide system;
        this.linearSlideServo = new ServoController(hardwareMap, Constants.LINEAR_SLIDE_SERVO);

        //  Servo attached to linear arm system;
        this.linearArmServo = new ServoController(hardwareMap, Constants.LINEAR_ARM_SERVO);
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


        if (gamepad.y) {
            this.pickup();
        }

        if (gamepad.a) {
            this.backToBase();
        }


        if (gamepad.x) {
            this.linearSlideServo.setPosition(0);
        }

        if (gamepad.b) {
            this.linearSlideServo.setPosition(1.0);
        }


        //  open, close
        this.linearArmServo.linearSlideArmTriggerPosition(gamepad.dpad_left, gamepad.dpad_right);

        double[] r = { this.linearSlideMotor.getCurrentPosition(), this.linearArmServo.getPosition(), this.linearSlideServo.getPosition() };
        return r;
    }

    public int lift(double distanceInInches) {
        double distance = distanceInInches - this.linearSlideMotor.calculateInches(this.linearSlideMotor.getCurrentPosition());
        double pow = distance > 0 ? POWER_LEVEL : -POWER_LEVEL;
        return this.linearSlideMotor.drive(distance, pow);
    }


    public int backToBase() {
        this.moveForwardSlide();

        //  Go back to starting position;
        return this.lift(0);
    }

    public void pickup() {
        // Put lift at 11.22 inches;
        this.lift(11.22);
        this.moveBackSlide();
        this.releaseObject();

        //  Double check;
        this.lift(8.987);
        this.grabObject();

        this.lift(11.22);
        this.backToBase();
    }

    public void moveBackSlide() {
        this.linearSlideServo.setPositionByDegrees(0);
    }

    public void moveForwardSlide() {
        this.linearSlideServo.setPositionByDegrees(180);
    }

    public void grabObject() {
        this.linearArmServo.setPositionByDegrees(180);
    }

    public void releaseObject() {
        this.linearArmServo.setPositionByDegrees(0);
    }

    public void reset() {
        this.linearSlideMotor.stop();
        this.backToBase();
    }

}
