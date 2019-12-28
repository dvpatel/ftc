package org.blueprint.ftc.core;

public class LiftSystem {

    private GameBot rosie;

    private SimpleMotor linearSlideMotor;
    private ServoController linearSlideServo;
    private ServoController linearArmServo;

    private static final int DISTANCE_IN_INCHES = 24;
    private static final double POWER_LEVEL = 0.90;

    private int currentPosition;

    public LiftSystem(GameBot rosie) {
        //  Make sure init is called from Tele / Autonomous code
        this.rosie = rosie;

        this.linearSlideMotor = this.rosie.getLinearSlideMotor();
        this.linearSlideServo = this.rosie.getLinearSlideServo();
        this.linearArmServo = this.rosie.getLinearArmServo();

        //  Reset to base
        this.backToBase();
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
