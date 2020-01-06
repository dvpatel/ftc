package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.Constants;

@TeleOp(name = "GyroCali", group = "Tele")
@Disabled
public class GyroTele extends AbstractLinearOpMode {

    private double angleCorrection;

    private static final double VELOCITY = 0.30 * Constants.MOTOR_MAX_VELOCITY;

    //  Delete in future ;
    private void waitForCalibration() {
        // make sure the imu gyro is calibrated before continuing.
        //  Automatically calibrated, no need to do this for standard mode
        //  while (!isStopRequested() && !this.rosie.getIMUController().isCalibrated()) {
        //     sleep(50);
        //      idle();
        //  }

    }

    @Override
    public void initOpMode() throws InterruptedException {

        this.initRosie();
        //  this.waitForCalibration(); //  Rosie takes care of calibration ;

        telemetry.addData("Calibration Status:  ", this.rosie.getIMUController().getCalibrationStatus());

    }

    @Override
    public void stopOpMode() {

    }

    @Override
    public void runOpMode() throws InterruptedException {

        this.initOpMode();
        this.waitForStart();


        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Wait for play", this.rosie.getTimer().toString());
        }

        this.rosie.getTimer().reset();
        while (opModeIsActive()) {
            this.driveStraight();

            telemetry.addData("imu heading", this.rosie.getIMUController().getFirstAngle());
            telemetry.addData("global heading", this.rosie.getIMUController().getGlobalAngle());

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            this.rotate(-90, 0);

            telemetry.update();
        }

        //  Stop logic ;
    }

    private void driveStraight() {

        //  index 0:  leftPowerCorrection
        //  index 1:  rightPowerCorrection
        //  index 2:  correction value ;

        telemetry.addData("Velocity", VELOCITY);
        telemetry.addData("turn rotation", this.rosie.getIMUController().getAngle());

        double[] correction = this.rosie.getMotorPID().calculateDriveCorrection(VELOCITY, this.rosie.getIMUController().getAngle());
        telemetry.addData("correction", correction[2]);

        angleCorrection = correction[2];

    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double targetAngle) {
        // restart imu angle tracking.
        this.rosie.getIMUController().resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359)
            degrees = (int) Math.copySign(359, degrees);


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && this.rosie.getIMUController().getAngle() == targetAngle) {
                telemetry.addData("Going straight", this.rosie.getIMUController().getAngle());
                sleep(100);
            }

            do {
                double p = this.rosie.getMotorPID().calculateRotateCorrection(degrees, this.rosie.getIMUController().getAngle(), VELOCITY);
                telemetry.addData("-VelocityAdjustment", p);

            } while (opModeIsActive() && !this.rosie.getMotorPID().angleOnTarget());
        } else    // left turn.
            do {
                double p = this.rosie.getMotorPID().calculateRotateCorrection(degrees, this.rosie.getIMUController().getAngle(), VELOCITY);
                telemetry.addData("+VelocityAdjustment", p);
            } while (opModeIsActive() && !this.rosie.getMotorPID().angleOnTarget());


        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        this.rosie.getIMUController().resetAngle();
    }
}