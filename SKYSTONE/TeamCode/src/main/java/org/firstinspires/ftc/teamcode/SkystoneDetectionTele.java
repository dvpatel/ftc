package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.blueprint.ftc.core.ColorSensorController;
import org.blueprint.ftc.core.Constants;
import org.blueprint.ftc.core.Driver;
import org.blueprint.ftc.core.FoundationSystem;
import org.blueprint.ftc.core.IMUController;
import org.blueprint.ftc.core.IntakeSystem;
import org.blueprint.ftc.core.SkystoneDetector;
import org.blueprint.ftc.core.StoneSystem;

import static org.blueprint.ftc.core.Constants.TURN_SPEED;

//  See
//  https://github.com/gearsincorg/FTCVuforiaDemo/blob/master/TeleopOpmode.java

@Autonomous(name = "SkystoneDetection")
//  @Disabled
public class SkystoneDetectionTele extends BaseAutonomous {

    private SkystoneDetector skystoneDetector;
    private IntakeSystem intakeSystem;
    private StoneSystem stoneSystem;
    private FoundationSystem foundationSystem;
    private ColorSensorController colorSensor;
    private Driver driver;


    private boolean isBlue;

    double[] targetCoordinates;
    int detectedStoneNumber;

    //  Game quadrant;
    private GameQuadrant quadrant;

    private boolean quadrantSelected;

    private int foundationOffset;

    @Override
    public void initOpMode() throws InterruptedException {
        this.initRosie();

        this.driver = this.rosie.getDriver();

        this.foundationSystem = this.rosie.getFoundationSystem();

        this.colorSensor = this.rosie.getColorSensorController();

        this.stoneSystem = this.rosie.getStoneSystem();
        this.stoneSystem.setLinearOpMode(this);

        this.intakeSystem = this.rosie.getIntakeSystem();

        this.skystoneDetector = this.rosie.getSkystoneDetector();

        this.setupZone();
        detectedStoneNumber = this.skystoneDetector.detectSkystoneOnField(this, isBlue);
        targetCoordinates = this.skystoneDetector.getTargetCoordinatesInInches();
    }

    private void setupZone() throws InterruptedException {
        //  Select quadrant;
        this.quadrant = this.selectGameQuadrant();
        telemetry.addData("Selected Zone:  ", quadrant.toString());
        telemetry.addData("Robot Direction (- left, + right):  ", GameQuadrant.direction(quadrant));
        telemetry.addData("Mode:  ", "init complete;  Running");
        telemetry.update();

        switch (quadrant) {
            case LOADING_RED:
                isBlue = false;
                break;
            case LOADING_BLUE:
                isBlue = true;
                break;
            case BUILDING_BLUE:
            case BUILDING_RED:
        }
    }

    @Override
    public void stopOpMode() {
        this.intakeSystem.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.initOpMode();

        telemetry.addData("Stone Number", detectedStoneNumber);
        telemetry.update();

        this.waitToPressStart();

        //  Intake system too high, also can't be down before start of match or in Init.  Refs don't like this.
        this.intakeSystem.setIntakeServosInitPosition();

        //  Force value
        if (opModeIsActive()) {

            if (isBlue) {
                this.blueMission(detectedStoneNumber);
            } else {
                this.redMission(detectedStoneNumber);
            }

        }

        this.stopOpMode();
    }

    private void blueMission(int stoneNumber) {

        this.stoneSystem.positionSystem();

        goToStone("BlueLoadingStone" + stoneNumber);

        pickupStone();

        driveToFoundation();

        dropStone();

        positionFoundation();

        driveToColorline();
    }

    private void redMission(int stoneNumber) {

        this.stoneSystem.positionSystem();

        goToStone("RedLoadingStone" + stoneNumber);

        pickupStone();

        driveToFoundation();

        dropStone();

        positionFoundation();

        driveToColorline();

    }

    private void goToStone(String trajName) {
        double dX = Math.abs(targetCoordinates[0]);
        //  double dY = targetCoordinates[1];

        double dY = 0.0;
        switch (detectedStoneNumber) {
            case 1:
                dY = -0.75;  //  center of stone 1
                foundationOffset = 0;
                break;
            case 2:
                dY = 3.25;   //  center of stone 2;
                foundationOffset = 12;
                break;
            case 3:
                dY = 10.30;  //  center of stone 3;
                foundationOffset = 20;
        }


        if (isBlue) {
            if (dY > 0) {
                this.driveForward(dY, (int) (0.45 * Constants.MOTOR_MAX_VELOCITY), false);
            } else {
                this.driveReverse(-dY, (int) (0.45 * Constants.MOTOR_MAX_VELOCITY), false);
            }

            sleep(250);
        } else {

            if (dY > 0) {
                this.driveReverse(dY, (int) (0.45 * Constants.MOTOR_MAX_VELOCITY), false);
            } else {
                this.driveForward(-dY, (int) (0.45 * Constants.MOTOR_MAX_VELOCITY), false);
            }

            sleep(250);
        }

        this.strafeLeft(dX, 0.40 * Constants.MOTOR_MAX_VELOCITY);
        sleep(50);

    }

    private void pickupStone() {

        this.stoneSystem.pickupStone();
        sleep(50);

        this.strafeRight(8.0, 0.25 * Constants.MOTOR_MAX_VELOCITY);

        sleep(50);

    }

    private void driveToFoundation() {

        if (isBlue) {
            this.driveReverse((80 + foundationOffset));

        } else {
            this.driveForward((80 + foundationOffset));
        }

        sleep(50);
        this.strafeLeft(9, 0.25 * Constants.MOTOR_MAX_VELOCITY);

    }

    private void dropStone() {

        //  Deposit;
        this.stoneSystem.putdownStone();
        sleep(100);

        //if foundation system touches stone, you may have to increase forward/reverse distance

        if (isBlue) {
            this.driveForward(5, (int) (0.45 * Constants.MOTOR_MAX_VELOCITY), false);
        } else {
            this.driveReverse(5, (int) (0.45 * Constants.MOTOR_MAX_VELOCITY), false);
        }
        sleep(50);

        this.strafeRight(5, 0.25 * Constants.MOTOR_MAX_VELOCITY);
        sleep(50);

        this.turnRight(90, TURN_SPEED);
        sleep(50);

        this.driveReverse(5, (int) (0.30 * Constants.MOTOR_MAX_VELOCITY), false);
        sleep(500);
    }

    private void positionFoundation() {

        this.foundationSystem.triggerDown(true);
        sleep(250);

        if (isBlue) {

            this.strafeLeft(8, 0.75 * Constants.MOTOR_MAX_VELOCITY);
            sleep(50);

            this.driveForward(12, (int) (0.45 * Constants.MOTOR_MAX_VELOCITY), false);
            sleep(50);

            this.turnLeft(90, Constants.MOTOR_MAX_VELOCITY);
            sleep(90);

        } else {

            this.strafeRight(8, 0.75 * Constants.MOTOR_MAX_VELOCITY);
            sleep(50);

            this.driveForward(12, (int) (0.45 * Constants.MOTOR_MAX_VELOCITY), false);
            sleep(50);

            this.turnRight(90, Constants.MOTOR_MAX_VELOCITY);
            sleep(50);

        }

        this.driveReverse(4, (int) (0.45 * Constants.MOTOR_MAX_VELOCITY), false);
        sleep(50);

        this.foundationSystem.triggerUp(true);
        sleep(50);

        this.stoneSystem.closeGripper();

    }

    private void driveToColorline() {

        //  Direction will return +1 or -1 based on quadrant selection;
        //  positive velocity strafe to right; negative strafe to the left;
        //  NOTE:  Do NOT use max velocity; skips color sensor detection;

        //  Try with both..
        //  Default:  positive velocity strafe right

        int ticks = driver.calculateTicks(36);

        this.drive(650);
        while (opModeIsActive()
                && !(colorSensor.isTargetBlue() || colorSensor.isTargetRed())
                && driver.getCurrentPosition()[0] <= ticks ) {

            telemetry.addData("ColorNotFound", "True");
            telemetry.update();

            //  Required
            //  to give other component chance to execute.
            idle();
        }
        this.driver.stop();

        this.driveReverse(6, (int) (0.30 * Constants.MOTOR_MAX_VELOCITY), false);

        this.stopOpMode();

    }
}
