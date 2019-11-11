package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

//  Alg based on http://controls.coderedrobotics.com/programminglessons/11.html
//  Drive using Gamepad ;
public class GamepadDriver {

    private Driver driver;

    double leftF, leftB, rightF, rightB;

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

        leftF = left_stick_y;
        rightF = left_stick_y;
        leftB = left_stick_y;
        rightB = left_stick_y;

        leftF += left_stick_x;
        rightF += -left_stick_x;

        leftB += -left_stick_x;
        rightB += left_stick_x;

        leftF += right_stick_x;
        rightF += -right_stick_x;
        leftB += right_stick_x;
        rightB += -right_stick_x;

        double max = Math.max(Math.max(Math.abs(leftF), Math.abs(rightF)), Math.max(Math.abs(leftB), Math.abs(rightB)));

        if (max > 1) {
            leftF = leftF / max;
            rightF = rightF / max;
            leftB = leftB / max;
            rightB = rightB / max;
        }

        //  Double check direction ;
        double[] result = {-leftF, -rightF, -leftB, -rightB};
        return result;
    }


}
