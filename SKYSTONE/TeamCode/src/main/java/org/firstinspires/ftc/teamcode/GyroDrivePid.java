//  https://stemrobotics.cs.pdx.edu/node/7268
//
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "GyroDrivePid", group = "Auto")
//@Disabled
public class GyroDrivePid extends AbstractLinearOpMode {

    private MotorControllerEx motor;
    private Driver driver;
    private IMUController imu;

    double power = .30;

    @Override
    void initOpMode() throws InterruptedException {
        telemetry.addData("Mode", "init Rosie");
        telemetry.update();

        this.initRosie();

        this.imu = this.rosie.getIMUController();
        telemetry.addData("Calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();

        this.motor = this.rosie.getMotorPID();  //  Defaut power ;
        this.driver = this.rosie.getDriver();

        telemetry.addData("Mode", "initOpMode Done");
        telemetry.update();
    }

    @Override
    void stopOpMode() {
        driver.stop();
        imu.resetAngle();
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        imu.resetAngle();

        // wait for start button.
        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();
        sleep(1000);

        while (opModeIsActive()) {
            // Use PID with imu input to drive in a straight line.
            double angle = imu.getAngle();
            double[] correction = motor.calculateDriveCorrection(power, angle);

            if (angle != 0) {
                telemetry.addData("Angle:  ", angle);
                telemetry.addData("P0:  ", correction[0]);
                telemetry.addData("P1:  ", correction[1]);
                telemetry.update();
            }

            driver.driveDifferential(correction[0], correction[1]);
        }

        this.stopOpMode();
    }
}