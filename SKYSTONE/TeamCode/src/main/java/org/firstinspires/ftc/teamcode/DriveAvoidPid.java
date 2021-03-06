// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses PID controller to drive in a straight line when not
// avoiding an obstacle.
//
// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.


//  From:
//  https://stemrobotics.cs.pdx.edu/node/7268

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Drive Avoid PID", group="Exercises")
@Disabled
public class DriveAvoidPid extends LinearOpMode
{

    private MotorControllerEx motor ;
    private Driver driver ;

    private TouchSensorController touchSensor ;
    private IMUController imu ;

    //  private double rotation ;
    private boolean aButton, bButton, touched;

    private double angleCorrection ;

    private void initDevices() {

        this.driver = new Driver(hardwareMap);
        this.motor = new MotorControllerEx() ;

        this.imu = new IMUController(hardwareMap) ;

        this.waitForCalibration();

        telemetry.addData("Mode", "init complete");
        telemetry.update();
    }

    private void waitForCalibration() {
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calibration status", this.imu.getCalibrationStatus());
        telemetry.update();
    }

    private void waitToPressStart() {
        // wait for start button.
        waitForStart();

        //  why is this needed?
        sleep(1000);
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        this.initDevices();
        this.waitToPressStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        double power = 0.30;

        //  Enable PID Controller to track state
        //  this.motor.enablePID();
        this.motor.enableDrivePID(power);

        // drive until end of period.
        while (opModeIsActive())
        {
            this.driveStraight(power);

            telemetry.addData("1 imu heading", this.imu.getFirstAngle());
            telemetry.addData("2 global heading", this.imu.getGlobalAngle());
            telemetry.addData("3 correction", angleCorrection);
            telemetry.addData("4 turn rotation", this.imu.getAngle());
            telemetry.update();

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            aButton = gamepad1.a;
            bButton = gamepad1.b;
            touched = this.touchSensor.pressed();

            if (touched || aButton || bButton)
            {

                this.driveReverse(power);

                // turn 90 degrees right.
                if (touched || aButton) rotate(-90, power, 0);

                // turn 90 degrees left.
                if (bButton) rotate(90, power, 0);
            }
        }

        driver.stop();
    }

    private void driveReverse(double power) {

        // backup.
        //  double[] reverseCorrection = this.motor.calculateDriveCorrection(-power, this.imu.getAngle());
        //  driver.driveDifferential(reverseCorrection[0], reverseCorrection[1]);
        driver.drive(-power);

        //  Reverse for 500 mSec.
        sleep(500);

        driver.stop();

    }

    private void driveStraight(double power) {

        //  index 0:  leftPowerCorrection
        //  index 1:  rightPowerCorrection
        //  index 2:  correction value ;


        double[] correction = this.motor.calculateDriveCorrection(power, this.imu.getAngle());
        driver.driveDifferential(correction[0], correction[1]);
        angleCorrection = correction[2] ;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power, double targetAngle)
    {
        // restart imu angle tracking.
        this.imu.resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359)
            degrees = (int) Math.copySign(359, degrees);


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && this.imu.getAngle() == targetAngle)
            {
                driver.driveDifferential(power, -power);
                sleep(100);
            }

            do
            {
                double powerCorrection = this.motor.calculateRotateCorrection(degrees, this.imu.getAngle(), power) ;
                driver.driveDifferential(powerCorrection, -powerCorrection);
            } while (opModeIsActive() && !this.motor.angleOnTarget());
        }
        else    // left turn.
            do
            {
                double powerCorrection = this.motor.calculateRotateCorrection(degrees, this.imu.getAngle(), power) ;
                driver.driveDifferential(-powerCorrection, powerCorrection);
            } while (opModeIsActive() && !this.motor.angleOnTarget());

        // turn the motors off.
        driver.stop();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        this.imu.resetAngle();
    }
}
