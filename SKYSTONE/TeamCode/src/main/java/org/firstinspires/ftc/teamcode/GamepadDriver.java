package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

//  left_stick_y:  -1.0 to 1.0 float;
//  left_stick_x:  -1.0 to 1.0 float;
//  Alg based on http://controls.coderedrobotics.com/programminglessons/11.html
//  Drive using Gamepad ;
public class GamepadDriver {

    private Driver driver;

    double leftFront, leftBack, rightFront, rightBack;

    public GamepadDriver(Driver driver) {
        this.driver = driver;
    }

    public double[] calculatePowerDifferential(Gamepad gamepad) {
        return this.calculatePowerDifferential(gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
    }

    public void drive(Gamepad gamepad) {
        double[] p = this.calculatePowerDifferential(gamepad);
        this.driver.powerDifferential(p[0], p[1], p[2], p[3]);
    }

    private double[] calculatePowerDifferential(float left_stick_y, float left_stick_x, float right_stick_x) {

        //  left_stick_y, left_stick_x:  Up = -1.0;  Down = 1.0;
        //               right_stick_x:  Up = -1.0;  Down: 1.0;

        leftFront = left_stick_y;
        rightFront = left_stick_y;
        leftBack = left_stick_y;
        rightBack = left_stick_y;

        leftFront += left_stick_x;
        rightFront += -left_stick_x;
        leftBack += left_stick_x;
        rightBack += -left_stick_x;

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
