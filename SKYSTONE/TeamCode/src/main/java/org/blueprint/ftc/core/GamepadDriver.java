package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.Gamepad;

import static org.blueprint.ftc.core.Constants.DEADZONE;

//  left_stick_y:  -1.0 to 1.0 float;
//  left_stick_x:  -1.0 to 1.0 float;
//  Alg based on http://controls.coderedrobotics.com/programminglessons/11.html
//  Vishnu:  https://drive.google.com/drive/u/1/folders/1Yb8Aari-lWgkvk-1ABZ3PhaJVYYrDe-S
//  Drive using GamePadTele ;
public class GamepadDriver {

    private Driver driver;
    private IMUController imu;

    private boolean goReverse = false;

    public GamepadDriver(Driver driver, IMUController imu) {
        this.imu = imu;
        this.driver = driver;
        this.driver.setRunWithEncoderOffMode();
    }

    private double toVelocity(double stickValue) {
        return stickValue * Constants.MOTOR_MAX_VELOCITY;
    }

    public void putInReverse(boolean inp) {

        if (inp) {
            this.goReverse = true;
        }

    }

    public void putInDrive(boolean inp) {

        if (inp) {
            this.goReverse = false;
        }

    }

    public boolean getDrivingDirection() {
        return this.goReverse;
    }

    public void driveTest(float driveForward, float sideways, float turn) {

        //  this.gpd.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        double leftFront = driveForward;
        double rightFront = driveForward;
        double leftBack = driveForward;
        double rightBack = driveForward;

        this.driver.velocityDifferential(
                toVelocity(leftFront),
                toVelocity(rightFront),
                toVelocity(leftBack),
                toVelocity(rightBack));

    }

    public void drive(float driveForward, float sideways, float turn) {

        double forward = this.getDrivingDirection() ? driveForward : -driveForward;

        //  Buffer;
        //  Needed if using gamepad.setJoystickDeadzone
        /*
        if (Math.abs(forward) < DEADZONE) forward = 0;
        if (Math.abs(sideways) < DEADZONE) sideways = 0;
        if (Math.abs(turn) < DEADZONE) turn = 0;
         */

        double leftFront = forward + sideways + turn;
        double rightFront = forward - sideways - turn;
        double leftBack = forward - sideways + turn;
        double rightBack = forward + sideways - turn;

        //  Scale;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(rightFront)), Math.max(Math.abs(leftBack), Math.abs(rightBack)));
        if (max > 1) {
            leftFront = leftFront / max;
            rightFront = rightFront / max;
            leftBack = leftBack / max;
            rightBack = rightBack / max;
        }

        this.driver.velocityDifferential(
                toVelocity(leftFront),
                toVelocity(rightFront),
                toVelocity(leftBack),
                toVelocity(rightBack));
    }
}
