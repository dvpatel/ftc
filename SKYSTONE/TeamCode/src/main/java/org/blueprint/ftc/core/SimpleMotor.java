package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.blueprint.ftc.core.Constants;

/**
 * Robot driver
 */
public class SimpleMotor {

    private DcMotor motor;

    public SimpleMotor(HardwareMap hardwareMap, String deviceName) {

        this.motor = hardwareMap.dcMotor.get(deviceName);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // this.motor.setDirection(DcMotor.Direction.REVERSE);

        //  this.setRunWithoutEncoderMode() ;
        this.power(0);
    }

    //  Calculate ticks for given inches ;
    public int calculateTicks(double distanceInInches) {
        return (int) (distanceInInches * Constants.SIMPLE_TICK_DIAMETER_RATIO);
    }

    private void setTargetPosition(double distanceInInches) {

        //  Always reset;  starts at zero;
        this.setStopAndResetMode();

        //  Run based on speed, not power;  OR run to target using position and power;
        //  this.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  Tells motor to run to target using position and power ;  Make sure t reset encoder when done!
        this.setRunWithoutEncoderMode();

        //  set target
        this.setTicksToTargets(distanceInInches);

        this.setRunToPositionMode();

        //  Apply power, somewhere ;  MAKE sure to turn off encoder when done.
    }

    //  Revisit;  Are all encoders needed?  Also should only one motor be used for calc?
    public boolean motorsBusy() {
        return this.motor.isBusy();
    }

    public void turnOffEncoders() {
        this.setStopAndResetMode();
        this.setRunWithoutEncoderMode();
    }

    public void setStopAndResetMode() {
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setRunWithoutEncoderMode() {
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setRunToPositionMode() {
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTicksToTargets(double distanceInInches) {
        int ticks = this.calculateTicks(distanceInInches);
        this.motor.setTargetPosition(ticks);
    }

    public void power(double power) {
        this.motor.setPower(power);
    }

    //  Current position in ticks;
    public int getCurrentPosition() {
        return this.motor.getCurrentPosition();
    }

    public void stop() {
        this.power(0);
    }

}