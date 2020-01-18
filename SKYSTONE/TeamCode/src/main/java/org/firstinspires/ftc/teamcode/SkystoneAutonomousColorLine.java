package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.blueprint.ftc.core.ColorSensorController;
import org.blueprint.ftc.core.Constants;
import org.blueprint.ftc.core.Driver;
import org.blueprint.ftc.core.SkystoneDetector;


// only color line;
@Autonomous(name = "ColorLineOnly", group = "Auto")
public class SkystoneAutonomousColorLine extends BaseAutonomous {

    private Driver driver;
    private ColorSensorController colorSensor;
    private SkystoneDetector skystoneDetector;

    private float mmPerInch;

    private static final int SLEEP_TIME = 100;

    //  Game quadrant;
    private GameQuadrant quadrant;

    private boolean quadrantSelected;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    protected void initOpMode() throws InterruptedException {

        this.initRosie();
        this.driver = this.rosie.getDriver();
        this.colorSensor = this.rosie.getColorSensorController();

        this.skystoneDetector = this.rosie.getSkystoneDetector();
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

        //  Put steps here;
        //  Drive to color;  set driving directions based on quadrant;
        this.driveToColorLine();

        //  Stop, done
        this.stopOpMode();
    }


    //  Strafe left or right based on quadrant to red or blue color
    private void driveToColorLine() {

        //  Direction will return +1 or -1 based on quadrant selection;
        //  positive velocity strafe to right; negative strafe to the left;
        //  NOTE:  Do NOT use max velocity; skips color sensor detection;
        double strafeVelocity = GameQuadrant.direction(quadrant) * (0.5 * Constants.MOTOR_MAX_VELOCITY);

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