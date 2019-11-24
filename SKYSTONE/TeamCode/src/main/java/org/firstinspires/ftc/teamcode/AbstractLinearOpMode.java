package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public abstract class AbstractLinearOpMode extends LinearOpMode {

    abstract void initOpMode() throws InterruptedException;

    abstract void stopOpMode();

    protected GameBot rosie;

    protected void waitToPressStart() {
        // wait for start button.
        waitForStart();

        //  why is this needed?
        sleep(1000);
    }

    protected void initRosie() throws InterruptedException {
        this.rosie = new GameBot();
        this.rosie.init(hardwareMap);
    }

    protected double normalizePower(double power) {
        return Range.clip(power, -1.0, 1.0);
    }

    protected void stopDriving() {
        this.rosie.getDriver().stop();
    }


    protected void driveForward(int distance, double power) {
        this.drive(distance, power);
    }

    protected void driveReverse(int distance, double power) {
        //  Both power and distance needs to be negative to go reverse
        this.drive(-distance, -power);
    }

    protected void strafeLeft(int distance, double power) {
        this.strafe(distance, power);
    }

    protected void strafeRight(int distance, double power) {
        //  Note:  both values must be negative ;
        this.strafe(-distance, -power);
    }

    protected void turnRight(int angle, double power) {
        this.turn(angle, power);
    }

    protected void turnLeft(int angle, double power) {
        //  note degrees direction and power
        this.turn(-angle, power);
    }


    protected void drive(double power) {
        Driver driver = this.rosie.getDriver();
        this.gyroDrive(power);
    }

    //  GyroDrive
    protected void drive(double distanceInInches, double power) {
        Driver driver = this.rosie.getDriver();
        driver.setTargetPosition(distanceInInches);

        this.rosie.getIMUController().resetAngle();

        do {
            this.gyroDrive(power);
        } while (opModeIsActive() && driver.motorsBusy());

        driver.stop();

        //  Disable encoders ;
        driver.turnOffEncoders();
    }

    protected void rotate(double power) {
        Driver driver = this.rosie.getDriver();
        driver.rotate(power);
    }

    protected void strafe(double power) {
        Driver driver = this.rosie.getDriver();
        this.gyroStrafe(power);
    }

    protected void turn(double degrees, double power) {

        Driver driver = this.rosie.getDriver();
        MotorControllerEx motor = this.rosie.getMotorPID();
        IMUController imu = this.rosie.getIMUController();

        imu.resetAngle();

        do {
            double[] p = motor.calculateRotateCorrection(degrees, imu.getAngle(), power);

            if (degrees < 0) {
                driver.powerDifferential(p[0], -p[1], p[0], -p[1]);

            } else {
                driver.powerDifferential(-p[0], p[1], -p[0], p[1]);
            }

        } while (opModeIsActive() && !motor.angleOnTarget());

        driver.stop();
        driver.turnOffEncoders();
    }

    protected void strafe(double distanceInInches, double power) {

        Driver driver = this.rosie.getDriver();
        driver.setTargetPosition(distanceInInches);

        this.rosie.getIMUController().resetAngle();

        do {
            this.gyroStrafe(power);
        } while (opModeIsActive() && driver.motorsBusy());

        driver.stop();
        driver.turnOffEncoders();
    }

    protected void gyroDrive(double power) {

        Driver driver = this.rosie.getDriver();
        MotorControllerEx motor = this.rosie.getMotorPID();
        IMUController imu = this.rosie.getIMUController();

        //  index 0:  leftPowerCorrection
        //  index 1:  rightPowerCorrection
        //  index 2:  correction value ;
        double[] correction = motor.calculateDriveCorrection(power, imu.getAngle());

        boolean showDebug = correction[0] != 0 || correction[1] != 0;
        driver.driveDifferential(this.normalizePower(correction[0]), this.normalizePower(correction[1]));

        if (showDebug) {
            telemetry.addData("Non-Zero Angle.  Adjust power.", imu.getAngle());
            telemetry.update();
        }
    }

    //  positive power strafe left; negative strafe right
    protected void gyroStrafe(double power) {

        Driver driver = this.rosie.getDriver();
        MotorControllerEx motor = this.rosie.getMotorPID();
        IMUController imu = this.rosie.getIMUController();

        //  index 0:  leftPowerCorrection
        //  index 1:  rightPowerCorrection
        //  index 2:  correction value ;
        double[] correction = motor.calculateDriveCorrection(power, imu.getAngle());
        boolean showDebug = correction[0] != 0 || correction[1] != 0;

        driver.strafeDifferential(this.normalizePower(correction[0]), this.normalizePower(correction[1]));

        if (showDebug) {
            telemetry.addData("Non-Zero Angle.  Adjust power.", imu.getAngle());
            telemetry.update();
        }
    }
}

