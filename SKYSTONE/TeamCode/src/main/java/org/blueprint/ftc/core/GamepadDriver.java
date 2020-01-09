package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.blueprint.ftc.core.Driver;
import org.blueprint.ftc.core.GameBot;

import static org.blueprint.ftc.core.Constants.DEADZONE;

//  left_stick_y:  -1.0 to 1.0 float;
//  left_stick_x:  -1.0 to 1.0 float;
//  Alg based on http://controls.coderedrobotics.com/programminglessons/11.html
//  Vishnu:  https://drive.google.com/drive/u/1/folders/1Yb8Aari-lWgkvk-1ABZ3PhaJVYYrDe-S
//  Drive using GamePadTele ;
public class GamepadDriver {

    private Driver driver;

    double leftFront, leftBack, rightFront, rightBack;

    GameBot game = new GameBot();

    public GamepadDriver(Driver driver) {
        this.driver = driver;
        this.driver.setRunWithEncoderOffMode();
    }

    private double toVelocity(double stickValue) {
        return stickValue * Constants.MOTOR_MAX_VELOCITY;
    }

    public void drive(Gamepad gamepad) {

        double forward = -gamepad.left_stick_y;
        double sideways = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        //  Buffer;
        if (Math.abs(forward) < DEADZONE) forward = 0;
        if (Math.abs(sideways) < DEADZONE) sideways = 0;
        if (Math.abs(turn) < DEADZONE) turn = 0;

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
