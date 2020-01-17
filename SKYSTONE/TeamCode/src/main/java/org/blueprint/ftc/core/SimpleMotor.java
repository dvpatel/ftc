package org.blueprint.ftc.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //  Calculate ticks for given inches ;
    public int calculateTicks(double distanceInInches) {
        return (int) (distanceInInches * Constants.SIMPLE_TICK_DIAMETER_RATIO);
    }

    public int calculateInches(double ticks) {
        return (int) (ticks / Constants.SIMPLE_TICK_DIAMETER_RATIO);
    }

    //  Revisit;  Are all encoders needed?  Also should only one motor be used for calc?
    public boolean motorsBusy() {
        return this.motor.isBusy();
    }

    public void drive(double power) {
        this.motor.setPower(power);
    }

    public int drive(LinearOpMode opMode, double distanceInInches, double pow) {

        //  power direction makes slide go up or down;
        double deltaPos = distanceInInches-this.calculateInches(this.motor.getCurrentPosition());
        double power = Math.signum(deltaPos) * pow;

        // for reset, set deltaPos to 0;
        if (distanceInInches == 0) {
            deltaPos = 0;
            power = -1 * pow;
        }

        //  This is our target in ticks; increasing or decreasing based on power;
        int newPosition = Math.abs(this.calculateTicks(deltaPos));

        //  going to the right, current position increasing, positive power;
        this.drive(power);
        if (power > 0) {
            while(opMode.opModeIsActive() && this.motor.getCurrentPosition() <= newPosition) {
                opMode.telemetry.addData("Going Up:  ", this.motor.getCurrentPosition());
                opMode.telemetry.addData("Target:  ", newPosition);
                opMode.telemetry.update();
            }
        } else {
            while(opMode.opModeIsActive() && this.motor.getCurrentPosition() >= newPosition) {
                opMode.telemetry.addData("Going Down:  ", this.motor.getCurrentPosition());
                opMode.telemetry.addData("Target:  ", newPosition);
                opMode.telemetry.update();
            }
        }
         this.stop();
        return this.getCurrentPosition();
    }

    //  Current position in ticks;
    public int getCurrentPosition() {
        return this.motor.getCurrentPosition();
    }

    public void stop() {
        this.drive(0);
    }
}