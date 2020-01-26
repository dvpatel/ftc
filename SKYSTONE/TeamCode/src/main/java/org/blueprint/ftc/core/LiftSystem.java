package org.blueprint.ftc.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSystem {

    private LinearOpMode myOpMode;

    private SimpleMotor linearSlideMotor;
    private ServoController linearSlideServo;
    private ServoController linearArmServo;

    private static final double SLIDE_COUNTER_MAX = 50.0;
    private double slideCounter;

    private static final double POWER_LEVEL = 0.65;

    private static final double BACKWARD_POS = 0.9;
    private static final double FORWARD_POS = 0.259;   //  70/270

    public LiftSystem(HardwareMap hardwareMap) {

        //  motor to control linear slide system
        this.linearSlideMotor = new SimpleMotor(hardwareMap, Constants.LINEAR_SLIDE_MOTOR_NAME);

        //  Servo attached to linear slide system;
        this.linearSlideServo = new ServoController(hardwareMap, Constants.LINEAR_SLIDE_SERVO);

        //  Servo attached to linear arm system;
        this.linearArmServo = new ServoController(hardwareMap, Constants.LINEAR_ARM_SERVO);
    }

    public void setLinearOpMode(LinearOpMode myOpMode) {
        this.myOpMode = myOpMode;
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

    //  Used in tele with Gamepad;
    //  returns motor positio, arm position, slideservo position;

    public void drive(float power) {
        float yVal = power;

        //  If max or min height, stop;
        if (((this.linearSlideMotor.getCurrentPosition() > Constants.SIMPLE_WHEEL_MAX_TICKS) && yVal > 0) ||
                ((this.linearSlideMotor.getCurrentPosition() < 10) && yVal < 0)) {
            yVal = 0;
        }
        this.linearSlideMotor.drive(yVal);
    }

    public void positionSlideServo(float positionValue) {

        //  -1 to 1  -->  very sensitive
        //  -1000 to 1000;  Move slide;
        double inp = positionValue;
        if (inp > 0) {
            slideCounter++;

            if (slideCounter > SLIDE_COUNTER_MAX) {
                slideCounter = SLIDE_COUNTER_MAX;
            }

            double slidePos = slideCounter / SLIDE_COUNTER_MAX;
            this.linearSlideServo.setPosition(slidePos);

        } else if (inp < 0) {
            slideCounter--;
            if (slideCounter < -SLIDE_COUNTER_MAX) {
                slideCounter = -SLIDE_COUNTER_MAX;
            }

            double slidePos = slideCounter / SLIDE_COUNTER_MAX;
            this.linearSlideServo.setPosition(slidePos);
        }

    }

    public int lift(double targetDistanceInInches) {
        return this.linearSlideMotor.drive(myOpMode, targetDistanceInInches, POWER_LEVEL);
    }

    public int backToBase(boolean input) {

        if (input) {

            //  back:  0.9  (243 degrees)
            //  forward:  0.29 (78.3)
            //  down;
            //  this.linearSlideServo.setPosition(0.666);
            //  this.moveForwardSlide();

            //  Go back to starting position;
            this.lift(0);

        }

        return this.getCurrentPosition();
    }

    public int pickup(boolean input) {

        if (input) {

            //  Go back starting position;
            this.moveForwardSlide();
            this.lift(0);
            myOpMode.sleep(250);

            //  consider offset;  Start at 11.50";
            double startingPoint = 12.50;

            //  inches;
            this.lift(startingPoint);  //  POS: 8854;  Don't chnage this.
            this.moveBackSlide();
            this.releaseObject();
            myOpMode.sleep(750);

            //  Double check;
            //  Was -4.85;  Target is 6.65
//        this.lift(6.65);  //  POS:  ~7890;  Don't change this.
            this.lift(10.0);  //  POS:  ~7890;  Don't change this.
            myOpMode.sleep(500);

            this.grabObject();
            myOpMode.sleep(500);  //  Don't change this.

            //  Was 12.0
            this.lift(19.65);    //  POS:  ~9645
            myOpMode.sleep(750);

            this.moveForwardSlide();
            myOpMode.sleep(500);

            //  Back to base;
            this.lift(0);

            //  this.moveForwardSlide(0.40);  //  0.35*270 degrees
            this.moveForwardSlide(0.37);  //  0.35*270 degrees

        }

        return this.getCurrentPosition();
    }

    public int getCurrentPosition() {
        return this.linearSlideMotor.getCurrentPosition();
    }

    public void moveBackSlide() {
        this.linearSlideServo.setPosition(BACKWARD_POS);
    }

    public void moveForwardSlide() {
        this.linearSlideServo.setPosition(FORWARD_POS);
    }

    public void moveForwardSlide(double pos) {
        this.linearSlideServo.setPosition(pos);
    }

    public void grabObject() {
        this.linearArmServo.setPosition(1.0);
    }

    public void releaseObject() {
        this.linearArmServo.setPosition(0.0);
    }

    public void reset() {
        this.linearSlideMotor.stop();
        this.backToBase(true);
    }
}
