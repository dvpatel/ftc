package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.blueprint.ftc.core.Driver;
import org.blueprint.ftc.core.GameBot;

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
    }

    public double[] calculatePowerDifferential(Gamepad gamepad) {
        return this.calculatePowerDifferential(gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
    }

    public void drive(Gamepad gamepad) {
        double[] p = this.calculatePowerDifferential(gamepad);
//        this.driver.powerDifferential(p[0], p[1], p[2], p[3]);

        double straightBack = -gamepad.left_stick_y;
        double leftRight = gamepad.left_stick_x;
        double turnMann = gamepad.right_stick_x;

        //  Confirm with Vishnu;
        this.driver.powerDifferential(straightBack + leftRight + turnMann, straightBack - leftRight - turnMann, straightBack - leftRight + turnMann, straightBack + rightBack - turnMann);

    }

    private double[] calculatePowerDifferential(float left_stick_y, float left_stick_x, float right_stick_x) {

        //  left_stick_y, left_stick_x:  Up = -1.0;  Down = 1.0;
        //               right_stick_x:  Up = -1.0;  Down: 1.0;

        leftFront = -left_stick_y;
        rightFront = -left_stick_y;
        leftBack = -left_stick_y;
        rightBack = -left_stick_y;

        // leftFront = leftFront + left_stick_x
        leftFront += left_stick_x;

        // rightFront = rightFront - left_stick_x
        rightFront += -left_stick_x;

        // leftBack = leftBack + left_stick_x
        leftBack += -left_stick_x;

        // rightBack = rightBack - left_stick_x
        rightBack += left_stick_x;

        leftFront += right_stick_x;
        rightFront += -right_stick_x;
        leftBack += right_stick_x;
        rightBack += -right_stick_x;

        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(rightFront)), Math.max(Math.abs(leftBack), Math.abs(rightBack)));

        if (max > 1) {
            leftFront = leftFront / max;
            rightFront = rightFront / max;
            leftBack = leftBack / max;
            rightBack = rightBack / max;
        }

        //  Double check direction ;
        double[] result = {leftFront, rightFront, leftBack, rightBack};
        return result;
    }


}
