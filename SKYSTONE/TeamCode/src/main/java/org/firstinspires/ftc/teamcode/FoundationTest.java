package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.Constants;
import org.blueprint.ftc.core.Driver;
import org.blueprint.ftc.core.FoundationSystem;
import org.blueprint.ftc.core.IMUController;
import org.blueprint.ftc.core.MotorControllerEx;

@Config
@Autonomous()
//  @Disabled
public class FoundationTest extends AbstractLinearOpMode {

    private MotorControllerEx motor;
    private Driver driver;
    private IMUController imu;
    private FoundationSystem foundationSystem;

    public static int DISTANCE_IN_INCHES = 24;
    public static double VELOCITY = 0.50*Constants.DEFAULT_VELOCITY;
    public static int SLEEP_TIME = 1000;

    @Override
    public void initOpMode() throws InterruptedException {

        this.initRosie();

        this.imu = this.rosie.getIMUController();
        telemetry.addData("Calibration Status:", this.imu.getCalibrationStatus());

        this.motor = this.rosie.getMotorPID();
        this.driver = this.rosie.getDriver();

        this.foundationSystem = this.rosie.getFoundationSystem();
        this.foundationSystem.triggerUp(true);

        telemetry.addData("Mode", "init complete;  Running");
        telemetry.update();
    }

    @Override
    public void stopOpMode() {
        driver.stop();
        this.foundationSystem.triggerUp(true);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //  Put common init logic here
        this.initOpMode();


        telemetry.update();

        //  Activate opmode
        this.waitToPressStart();

        if(opModeIsActive()) {

            this.strafeLeft(DISTANCE_IN_INCHES, VELOCITY);
            sleep(500);



            this.strafeLeft(1.0, VELOCITY);
            sleep(500);
            this.foundationSystem.triggerDown(true);
            sleep(500);


            //  Strafe 12 inches right ; both values must be negative ;
            this.strafeRight(DISTANCE_IN_INCHES, VELOCITY);
            sleep(SLEEP_TIME);


            this.strafeLeft(1.0, VELOCITY);
            sleep(250);
            this.foundationSystem.triggerUp(true);
        }

        this.stopOpMode();
    }
}