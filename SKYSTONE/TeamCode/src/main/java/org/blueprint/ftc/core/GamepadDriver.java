package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.blueprint.ftc.core.Driver;
import org.blueprint.ftc.core.GameBot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.blueprint.ftc.core.Constants.DEADZONE;

//  left_stick_y:  -1.0 to 1.0 float;
//  left_stick_x:  -1.0 to 1.0 float;
//  Alg based on http://controls.coderedrobotics.com/programminglessons/11.html
//  Vishnu:  https://drive.google.com/drive/u/1/folders/1Yb8Aari-lWgkvk-1ABZ3PhaJVYYrDe-S
//  Drive using GamePadTele ;
public class GamepadDriver {

    private Driver driver;
    private IMUController imu;
    double lastYawAngle;


    double leftFront, leftBack, rightFront, rightBack;

    GameBot game = new GameBot();

    public GamepadDriver(Driver driver, IMUController imu) {
        this.imu = imu;
        this.driver = driver;
        this.driver.setRunWithEncoderOffMode();
    }

    private double toVelocity(double stickValue) {
        return stickValue * Constants.MOTOR_MAX_VELOCITY;
    }

    public void drive(Gamepad gamepad) {
        this.holonomicDriving(gamepad);
        //  this.fieldOrientedDriving(gamepad);
    }

    //  Test;  Gears keep slipping!
    public void fieldOrientedDriving(Gamepad gamepad) {

        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        //  theta is yaw
        //  x cos(theta) -y sin(theta)  --->  strafe = strafe cos(theta) - forward sin(theta)
        //  x sin(theta) + y cos(theta) --->  forward = strafe sin(theta) + forward cos(theta)

        //  means turning;  only update when turning;
        double yawAngle = lastYawAngle;
        if (turn != 0) {
            yawAngle = this.imu.getAngle();   //  Accumulating
            //  double deltaAngle = yawAngle - lastYawAngle;
            //  if (Math.abs(deltaAngle) < Constants.ANGLE_DEADZONE)
            //    yawAngle = lastYawAngle;
        }


        strafe = strafe * Math.cos(yawAngle) - forward * Math.sin(yawAngle);
        forward = strafe * Math.sin(yawAngle) + forward * Math.cos(yawAngle);

        //  lastYawAngle = yawAngle;

        /* At this point, Joystick X/Y (strafe/forwrd) vectors have been */
        /* rotated by the gyro angle, and can be sent to drive system */

        //  Buffer;
        if (Math.abs(forward) < DEADZONE) forward = 0;
        if (Math.abs(strafe) < DEADZONE) strafe = 0;
        if (Math.abs(turn) < DEADZONE) turn = 0;

        double leftFront = forward + strafe + turn;
        double rightFront = forward - strafe - turn;
        double leftBack = forward - strafe + turn;
        double rightBack = forward + strafe - turn;

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

    public void holonomicDriving(Gamepad gamepad) {

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
