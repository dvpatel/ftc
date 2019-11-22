
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NeedhamRedBuilding", group = "Auto")
//  @Disabled
public class NeedhamAuto extends AbstractLinearOpMode {

    private MotorControllerEx motor;
    private Driver driver;
    private IMUController imu;
    private ServoController servo;
    private ColorSensorController colorSensor;

    private static final int DISTANCE_IN_INCHES = 12;
    private static final int TURN_ANGLE = 90;
    private static final int SLEEP_TIME = 500;
    private static final double POWER_LEVEL = 0.40;

    private static final float TILE_SIZE = 22.75f;  //  22.65 intches;

    @Override
    void initOpMode() throws InterruptedException {

        this.initRosie();

        this.imu = this.rosie.getIMUController();
        this.motor = this.rosie.getMotorPID();
        this.driver = this.rosie.getDriver();
        this.servo = this.rosie.getShortArmServo();
        this.colorSensor = this.rosie.getColorSensorController();


        telemetry.addData("Mode", "init complete;  Running");
        telemetry.update();
    }

    @Override
    void stopOpMode() {
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

        //  make sure power is between -1 and 1 ;
        double power = this.normalizePower(POWER_LEVEL);

        //  Start middle of 2 tiles in red zone;
        //  Note:  ShortArm is on the left side

        //  Go forward 5 inches;
        this.driveForward(12, power);
        sleep(SLEEP_TIME);

        //  Turn 90 degrees right @ 0.2 power ;
        this.turnRight(TURN_ANGLE, 0.2);
        sleep(SLEEP_TIME);

        //  Reverse:  2.5 tiles or 44.5 + 11.375 = 55.875 = 56;
        this.driveReverse(56, power);
        sleep(SLEEP_TIME);

        //  still 12" from wall ;  width:  44.5, therefore still 32.5: from skystone ;
        //  Strafe left ;
        this.strafeLeft(33, power);
        sleep(SLEEP_TIME);

        //  Try to get skystone:  leftTrigger, rightTrigger;
        //  TEST
        this.servo.triggerPosition(0, 0.5f);
        sleep(SLEEP_TIME);

        //  StrafeRight;
        this.strafeRight(33, power);
        sleep(SLEEP_TIME);

        //  Go forward ;  In building zone ;
        this.driveForward(56, power);
        sleep(SLEEP_TIME);

        //  ShortArm servo up ;
        this.servo.triggerPosition(0.5f, 0);
        sleep(SLEEP_TIME);

        //  Drive back ;  Puts us under bridge ;
        //  this.driveReverse(20, power);
        //  sleep(SLEEP_TIME);

        //  Or try color sensor ;  drive reverse with negative poewr ;
        while (opModeIsActive() && !(colorSensor.isTargetBlue() || colorSensor.isTargetRed())) {
            this.drive(-power);
        }

        //  Stop, done
        this.stopOpMode();
    }
}