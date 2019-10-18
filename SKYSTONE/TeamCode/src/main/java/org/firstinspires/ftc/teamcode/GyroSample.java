package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BasicGyroTest", group = "Linear Opmode")
//  @Disabled
public class GyroSample extends LinearOpMode {

    private MotorControllerEx motor;
    private IMUController imu;

    private double angleCorrection;

    double power = 0.30;


    private void initDevices() {

        this.motor = new MotorControllerEx();
        this.motor.enableDrivePID(power);

        this.imu = new IMUController(hardwareMap);
        this.waitForCalibration();
    }

    private void waitForCalibration() {
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calibration status", this.imu.getCalibrationStatus());
    }

    private void waitToPressStart() {
        // wait for start button.
        waitForStart();

        //  why is this needed?
        sleep(1000);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        this.initDevices();

        // wait for start button.
        this.waitForStart();

        while (opModeIsActive()) {
            this.driveStraight(power);

            telemetry.addData("imu heading", this.imu.getFirstAngle());
            telemetry.addData("global heading", this.imu.getGlobalAngle());

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            this.rotate(-90, power, 0);

            telemetry.update();
        }

        //  Stop logic ;
    }

    private void driveStraight(double power) {

        //  index 0:  leftPowerCorrection
        //  index 1:  rightPowerCorrection
        //  index 2:  correction value ;

        telemetry.addData("power", power);
        telemetry.addData("turn rotation", this.imu.getAngle());

        double[] correction = this.motor.calculateDriveCorrection(power, this.imu.getAngle());
        telemetry.addData("correction", correction[2]);

        angleCorrection = correction[2];

    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power, double targetAngle) {
        // restart imu angle tracking.
        this.imu.resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359)
            degrees = (int) Math.copySign(359, degrees);


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && this.imu.getAngle() == targetAngle) {
                telemetry.addData("Going straight", this.imu.getAngle());
                sleep(100);
            }

            do {
                double powerCorrection = this.motor.calculateRotateCorrection(degrees, this.imu.getAngle(), power);
                telemetry.addData("-PowerAdjustment", powerCorrection);

            } while (opModeIsActive() && !this.motor.angleOnTarget());
        } else    // left turn.
            do {
                double powerCorrection = this.motor.calculateRotateCorrection(degrees, this.imu.getAngle(), power);
                telemetry.addData("+PowerAdjustment", powerCorrection);
            } while (opModeIsActive() && !this.motor.angleOnTarget());


        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        this.imu.resetAngle();
    }
}