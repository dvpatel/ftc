package org.blueprint.ftc.core;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSystem {

    private LinearOpMode myOpMode;

    private SimpleMotor linearSlideMotor;
    private ServoController linearSlideServo;
    private ServoController linearArmServo;

    //  only for boolean buttons;
    private boolean pickupInProgress;
    private boolean backToBaseInProgress;
    private boolean XinProgress;
    private boolean BinProgress;

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

    public LinearOpMode getLinearOpMode() {

        if (this.myOpMode == null) {
            throw new NullPointerException("LinearOpMode not set.");
        }

        return this.myOpMode;
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

    private double slidePos = 0.0f;

    public double[] autoMode(Gamepad gamepad) {

        //  If max or min height, stop;
        float yVal = -gamepad.left_stick_y;
        if (((this.linearSlideMotor.getCurrentPosition() > Constants.SIMPLE_WHEEL_MAX_TICKS) && yVal > 0 ) ||
                    ((this.linearSlideMotor.getCurrentPosition() < 10) && yVal < 0) ) {
            yVal = 0;
        }
        this.linearSlideMotor.drive(yVal);

        if (gamepad.y && !this.pickupInProgress) {
            this.pickupInProgress = true;
            this.pickup();
            this.pickupInProgress = false;
        }

        if (gamepad.a && !this.backToBaseInProgress) {
            this.backToBaseInProgress = true;
            this.backToBase();
            this.backToBaseInProgress = false;
        }

        double inp = -gamepad.right_stick_y;
        if (inp > 0) {
            slidePos = slidePos + 0.001f;
            if (slidePos >= FORWARD_POS) {
                slidePos = FORWARD_POS;
            }

            this.linearSlideServo.setPosition(slidePos);

        } else if (inp < 0) {
            slidePos = slidePos - 0.001f;
            if (slidePos < BACKWARD_POS) {
                slidePos = BACKWARD_POS;
            }

            this.linearSlideServo.setPosition(slidePos);
        }



        if (gamepad.x && !this.XinProgress) {
            this.XinProgress = true;
            //  Push back;
            this.moveBackSlide();
            this.XinProgress = false;
        }

        if (gamepad.b && !this.BinProgress) {
            this.BinProgress = true;
            //  push out, 180 degrees or 2/3 of 270
            this.moveForwardSlide();
            this.BinProgress = false;
        }


        //  open, close
        this.linearArmServo.linearSlideArmTriggerPosition(gamepad.dpad_left, gamepad.dpad_right);
        double[] r = { this.linearSlideMotor.getCurrentPosition(), this.linearArmServo.getPosition(), this.linearSlideServo.getPosition() };
        return r;

    }

    public int lift(double targetDistanceInInches) {
        return this.linearSlideMotor.drive(myOpMode, targetDistanceInInches, POWER_LEVEL);
    }


    public int backToBase() {
        this.moveForwardSlide();

        //  Go back to starting position;
        this.lift(0);
        return this.linearSlideMotor.getCurrentPosition();
    }

    public int pickup() {

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
        this.lift(6.65);  //  POS:  ~7890;  Don't change this.
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
        this.moveForwardSlide(0.40);  //  0.35*270 degrees
        this.linearSlideMotor.stop();

        return this.linearSlideMotor.getCurrentPosition();
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
        this.backToBase();
    }

}
