package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.Constants;
import org.blueprint.ftc.core.Driver;
import org.blueprint.ftc.core.IMUController;
import org.blueprint.ftc.core.MotorControllerEx;

import static org.blueprint.ftc.core.Constants.TURN_SPEED;
import static org.blueprint.ftc.core.Constants.STRAFE_SPEED;

@Config
@Autonomous()
//  @Disabled
public class TargetDriveAuto extends AbstractLinearOpMode {

    private MotorControllerEx motor;
    private Driver driver;
    private IMUController imu;

    public static int DISTANCE_IN_INCHES = 36;
    public static int TURN_ANGLE = 90;
    public static int SLEEP_TIME = 5000;

    @Override
    public void initOpMode() throws InterruptedException {

        this.initRosie();

        this.imu = this.rosie.getIMUController();
        this.motor = this.rosie.getMotorPID();
        this.driver = this.rosie.getDriver();

        telemetry.addData("Mode", "init complete;  Running");
        telemetry.update();
    }

    @Override
    public void stopOpMode() {
        driver.stop();
    }

    private void addTelemetry() {

        telemetry.addData("Targeted Ticks:", driver.calculateTicks(DISTANCE_IN_INCHES));
        telemetry.addData("Targeted Inches:", DISTANCE_IN_INCHES);

        int[] currentPosition = driver.getCurrentPosition();
        telemetry.addData("CurrentPosition", "%d, %d, %d, %d",
                currentPosition[0], currentPosition[1], currentPosition[2], currentPosition[3]);

        telemetry.addData("CurrentPosition Inches", "%.04f, %.04f, %.0f, %.0f",
                driver.toInches(currentPosition[0]),
                driver.toInches(currentPosition[1]),
                driver.toInches(currentPosition[2]),
                driver.toInches(currentPosition[3]));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //  Put common init logic here
        this.initOpMode();

        telemetry.addData("Calibration Status:", this.imu.getCalibrationStatus());
        //  this.addTelemetry();
        telemetry.update();

        //  Activate opmode
        this.waitToPressStart();

        if(opModeIsActive()) {

            telemetry.addData("Forward:  ", "forward " + DISTANCE_IN_INCHES + " inches");
            telemetry.update();
            this.driveForward(DISTANCE_IN_INCHES);
            this.addTelemetry();
            telemetry.update();
            sleep(SLEEP_TIME);

            telemetry.addData("Strafe:  ", "strafe left " + DISTANCE_IN_INCHES + " inches");
            telemetry.update();
            this.strafeLeft(DISTANCE_IN_INCHES, STRAFE_SPEED);
            this.addTelemetry();
            telemetry.update();
            sleep(SLEEP_TIME);

            telemetry.addData("Reverse:  ", "reverse " + DISTANCE_IN_INCHES + " inches");
            telemetry.update();
            this.driveReverse(DISTANCE_IN_INCHES);
            this.addTelemetry();
            telemetry.update();
            sleep(SLEEP_TIME);

            telemetry.addData("Strafe:  ", "strafe right " + DISTANCE_IN_INCHES + " inches");
            telemetry.update();
            this.strafeRight(DISTANCE_IN_INCHES, STRAFE_SPEED);
            this.addTelemetry();
            telemetry.update();
            sleep(SLEEP_TIME);

            //  Turn 90 degrees to the right
            telemetry.addData("GyroTurn:  right ", "90 degrees");
            telemetry.update();
            this.turnRight(TURN_ANGLE, TURN_SPEED);
            telemetry.addData("GyroTurn:  Done, ", imu.getAngle());
            telemetry.update();
            sleep(SLEEP_TIME);

            //  Turn 90 degrees left;  note degrees direction and velocity
            telemetry.addData("GyroTurn: left  ", "90 degrees??");
            telemetry.update();
            this.turnLeft(TURN_ANGLE, TURN_SPEED);
            telemetry.addData("GyroTurn:  Done, ", imu.getAngle());
            telemetry.update();
            sleep(SLEEP_TIME);

            telemetry.addData("GyroTurn:  right ", "90X2 degrees");
            telemetry.update();
            this.turnRight(2 * TURN_ANGLE, TURN_SPEED);
            telemetry.addData("GyroTurn:  Done, ", imu.getAngle());
            telemetry.update();
            sleep(SLEEP_TIME);

            //  Turn 90 degrees left;  note degrees direction and velocity
            telemetry.addData("GyroTurn: left  ", "90X2 degrees??");
            telemetry.update();
            this.turnLeft(2 * TURN_ANGLE, TURN_SPEED);
            telemetry.addData("GyroTurn:  Done, ", imu.getAngle());
            telemetry.update();
            sleep(SLEEP_TIME);
        }

        this.stopOpMode();

        this.addTelemetry();
        telemetry.update();
    }
}