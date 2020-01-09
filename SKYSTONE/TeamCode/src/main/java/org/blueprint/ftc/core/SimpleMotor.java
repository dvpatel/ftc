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
        this.setStopAndResetMode();
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.drive(0);
    }

    //  Calculate ticks for given inches ;
    public int calculateTicks(double distanceInInches) {
        return (int) (distanceInInches * Constants.SIMPLE_TICK_DIAMETER_RATIO);
    }

    public void setTargetPosition(double distanceInInches) {

        //  Always reset;  starts at zero;
        this.setStopAndResetMode();

        //  set target
        this.setTicksToTargets(distanceInInches);

        //  Run to target position;
        this.setRunToPositionMode();

        //  Apply power, somewhere ;  MAKE sure to turn off encoder when done.
    }

    //  Revisit;  Are all encoders needed?  Also should only one motor be used for calc?
    public boolean motorsBusy() {
        return this.motor.isBusy();
    }

    public void setStopAndResetMode() {
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setRunToPositionMode() {
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTicksToTargets(double distanceInInches) {
        int ticks = this.calculateTicks(distanceInInches);
        this.motor.setTargetPosition(ticks);
    }

    public void drive(double power) {
        this.motor.setPower(power);
    }

    //  Current position in ticks;
    public int getCurrentPosition() {
        return this.motor.getCurrentPosition();
    }

    public void stop() {
        this.drive(0);
    }
}