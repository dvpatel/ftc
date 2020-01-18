package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.blueprint.ftc.core.ColorSensorController;
import org.blueprint.ftc.core.Constants;
import org.blueprint.ftc.core.Driver;
import org.blueprint.ftc.core.FoundationSystem;

@Autonomous(name = "FoundationAndColor", group = "Auto")
public class SkystoneAutonomousCombined extends BaseAutonomous {

    private Driver driver;
    private FoundationSystem foundationSystem;
    private ColorSensorController colorSensor;


    private float mmPerInch;

    private static final int SLEEP_TIME = 100;

    private static final int TURN_ANGLE = 90;

    private static final double STRAFE_SPEED = 0.25*Constants.MOTOR_MAX_VELOCITY;

    //  Game quadrant;
    private GameQuadrant quadrant;

    private boolean quadrantSelected;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    protected void initOpMode() throws InterruptedException {

        this.initRosie();
        this.driver = this.rosie.getDriver();
        this.foundationSystem = this.rosie.getFoundationSystem();
        this.colorSensor = this.rosie.getColorSensorController();

        mmPerInch = Constants.MM_PER_INCHES;

        //  Select quadrant;
        this.quadrant = this.selectGameQuadrant();
        telemetry.addData("Selected Zone:  ", quadrant.toString());
        telemetry.addData("Robot Direction (- left, + right):  ", GameQuadrant.direction(quadrant));
        telemetry.addData("Mode:  ", "init complete;  Running");
        telemetry.update();

        sleep(1000);
    }

    @Override
    protected void stopOpMode() {
        driver.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //  Put common init logic here
        this.initOpMode();

        //  Activate opmode
        this.waitToPressStart();

        telemetry.addData("Mode:  ", "Start pressed");
        telemetry.update();

        //  go forward to position at center of 2nd quadrant;
        this.driveForward(Constants.DRIVETRAIN_SECOND_QUADRANT_CENTER, Constants.DEFAULT_VELOCITY);
        sleep(SLEEP_TIME);

        switch (quadrant) {
            case BUILDING_BLUE:
                this.buildingBlueFoundation();
                break;
            case LOADING_BLUE:
                this.loadingBlueFoundation();
                break;
            case BUILDING_RED:
                buildingRedFoundation();
                break;
            case LOADING_RED:
                this.loadingRedFoundation();
        }

        //  Go to colored line;
        this.driveToColorLine();

        //  Stop, done
        this.stopOpMode();
    }

    private void commonBlueFoundationTask() {
        this.strafeRight(4.25, STRAFE_SPEED);
        this.foundationSystem.triggerDown();
        sleep(SLEEP_TIME);

        this.strafeLeft(7.5, STRAFE_SPEED);  //  puts robot at right edge of first tile, 24"
        sleep(SLEEP_TIME);

        //  move foundation parallael to front wall
        turnLeft(TURN_ANGLE, Constants.MOTOR_MAX_VELOCITY);
        sleep(SLEEP_TIME);

        //  puts us against the wall;  triggers up
        this.strafeLeft(10.25, STRAFE_SPEED);
        this.foundationSystem.triggerUp();
        sleep(SLEEP_TIME);

        //  release;
        this.strafeRight(10.25, STRAFE_SPEED);
        sleep(SLEEP_TIME);

        this.driveForward(3.25, Constants.MOTOR_MAX_VELOCITY);
        sleep(SLEEP_TIME);
    }

    public void buildingBlueFoundation() {

        this.turnRight(TURN_ANGLE, Constants.MOTOR_MAX_VELOCITY);
        sleep(SLEEP_TIME);

        this.driveReverse(24, Constants.MOTOR_MAX_VELOCITY);
        sleep(SLEEP_TIME);

        this.commonBlueFoundationTask();
    }

    public void loadingBlueFoundation() {

        this.turnRight(TURN_ANGLE, Constants.MOTOR_MAX_VELOCITY);
        sleep(SLEEP_TIME);

        this.driveReverse(72, Constants.MOTOR_MAX_VELOCITY);
        sleep(SLEEP_TIME);

        this.commonBlueFoundationTask();
    }

    public void commonRedFoundationTasks() {
        this.strafeLeft(4.25, STRAFE_SPEED);
        this.foundationSystem.triggerDown();
        sleep(SLEEP_TIME);

        this.strafeRight(7.5, STRAFE_SPEED);  //  puts robot at right edge of end tile
        sleep(SLEEP_TIME);

        //  move foundation parallael to front wall
        turnRight(TURN_ANGLE, Constants.MOTOR_MAX_VELOCITY);
        sleep(SLEEP_TIME);

        //  puts us against the wall;  triggers up
        this.strafeLeft(10.25, STRAFE_SPEED);
        this.foundationSystem.triggerUp();
        sleep(SLEEP_TIME);

        //  release;
        this.strafeRight(10.25, STRAFE_SPEED);
        sleep(SLEEP_TIME);

        this.driveReverse(3.25, Constants.MOTOR_MAX_VELOCITY);
        sleep(SLEEP_TIME);
    }

    public void buildingRedFoundation() {

        this.turnRight(TURN_ANGLE, Constants.MOTOR_MAX_VELOCITY);
        sleep(SLEEP_TIME);

        this.driveForward(24, Constants.MOTOR_MAX_VELOCITY);
        sleep(SLEEP_TIME);

        this.commonRedFoundationTasks();
    }

    public void loadingRedFoundation() {

        this.turnRight(TURN_ANGLE, Constants.MOTOR_MAX_VELOCITY);
        sleep(SLEEP_TIME);

        this.driveForward(72, Constants.MOTOR_MAX_VELOCITY);
        sleep(SLEEP_TIME);

        this.commonRedFoundationTasks();
    }


    private void driveToColorLine() {

        //  Direction will return +1 or -1 based on quadrant selection;
        //  positive velocity strafe to right; negative strafe to the left;
        //  NOTE:  Do NOT use max velocity; skips color sensor detection;
        double strafeVelocity = 0.5 * Constants.MOTOR_MAX_VELOCITY;

        //  Try with both..
        //  Default:  positive velocity strafe right

        runtime.reset();
        this.driver.strafe(strafeVelocity);
        while (opModeIsActive() && (runtime.seconds() < 5.0) && !(colorSensor.isTargetBlue() || colorSensor.isTargetRed())) {

            telemetry.addData("ColorNotFound", "True");
            telemetry.update();

            //  Required to give other component chance to execute.
            idle();
        }
        this.stopOpMode();
    }

}
