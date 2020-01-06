package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.Constants;
import org.blueprint.ftc.core.Driver;
import org.blueprint.ftc.core.IMUController;
import org.blueprint.ftc.core.MotorControllerEx;

@Autonomous(name = "TargetDrive", group = "Auto")
//  @Disabled
public class TargetDriveAuto extends AbstractLinearOpMode {

    private MotorControllerEx motor;
    private Driver driver;
    private IMUController imu;

    private static final int DISTANCE_IN_INCHES = 36;
    private static final int TURN_ANGLE = 90;
    private static final int SLEEP_TIME = 500;
    private static final double VELOCITY = Constants.DEFAULT_VELOCITY;

    @Override
    public void initOpMode() throws InterruptedException {

        this.initRosie();

        this.imu = this.rosie.getIMUController();
        this.motor = this.rosie.getMotorPID();
        this.driver = this.rosie.getDriver();

        //  Enable PID Controller to track state
        //  this.motor.enablePID();
        //this.motor.enableDrivePID(power);

        telemetry.addData("Mode", "init complete;  Running");
        telemetry.update();
    }

    @Override
    public void stopOpMode() {
        driver.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //  Put common init logic here
        this.initOpMode();

        telemetry.addData("Calibration Status:", this.imu.getCalibrationStatus());
        telemetry.update();

        //  Activate opmode
        this.waitToPressStart();

        telemetry.addData("Forward:  ", "forward");
        telemetry.update();
        this.driveForward(DISTANCE_IN_INCHES, VELOCITY);
        sleep(SLEEP_TIME);

        telemetry.addData("GyroStrafe:  ", "strafe left");
        telemetry.update();
        this.strafeLeft(24, VELOCITY);
        sleep(SLEEP_TIME);

        //  drive backward 12 inches ;
        telemetry.addData("Reverse:  ", "back 6 inches");
        telemetry.update();
        this.driveReverse(DISTANCE_IN_INCHES, VELOCITY);
        sleep(SLEEP_TIME);

        //  Strafe 12 inches right ; both values must be negative ;
        telemetry.addData("GyroStrafe:  ", "strafe 6 inches right");
        telemetry.update();
        this.strafeRight(24, VELOCITY);
        sleep(SLEEP_TIME);

        //  Turn 90 degrees to the right
        telemetry.addData("GyroTurn:  right ", "90 degrees");
        telemetry.update();
        this.turnRight(TURN_ANGLE, VELOCITY);
        telemetry.addData("GyroTurn:  Done, ", imu.getAngle());
        telemetry.update();
        sleep(SLEEP_TIME);

        //  Turn 90 degrees left;  note degrees direction and velocity
        telemetry.addData("GyroTurn: left  ", "90 degrees??");
        telemetry.update();
        this.turnLeft(TURN_ANGLE, VELOCITY);
        telemetry.addData("GyroTurn:  Done, ", imu.getAngle());
        telemetry.update();

    }
}