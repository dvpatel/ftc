package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.ColorSensorController;
import org.blueprint.ftc.core.Constants;
import org.blueprint.ftc.core.Driver;
import org.firstinspires.ftc.robotcore.external.Const;

@Autonomous(name = "SkystoneAutonomous", group = "Auto")
//  @Disabled
public class SkystoneAutonomous extends AbstractLinearOpMode {

    private Driver driver;
    private ColorSensorController colorSensor;

    private static final double DEFAULT_POWER = 0.25;
    private double strafePower;

    private static final int SLEEP_TIME = 100;

    //  Game quadrant;
    private GameQuadrant quadrant;

    @Override
    protected void initOpMode() throws InterruptedException {

        this.initRosie();
        this.driver = this.rosie.getDriver();
        this.colorSensor = this.rosie.getColorSensorController();

        //  Select quadrant;
        this.quadrant = this.selectGameQuadrant();
        telemetry.addData("Selected Zone:  ", quadrant.toString());
        telemetry.addData("Power Direction (- left, + right):  ", GameQuadrant.direction(quadrant));
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
        this.driveForward(Constants.TILE_SIZE + ((Constants.TILE_SIZE - Constants.ROBOT_LENGTH) / 2), DEFAULT_POWER);
        sleep(SLEEP_TIME);

        //  Put steps here;
        //  Drive to color;  set power directions based on quadrant;
        this.driveToColorLine();

        //  Stop, done
        this.stopOpMode();

    }

    //  Strafe left or right based on quadrant to red or blue color
    private void driveToColorLine() {

        //  Direction will return +1 or -1 based on quadrant selection;
        //  positive power strafe to right; negative strafe to the left;
        this.strafePower = GameQuadrant.direction(quadrant) * DEFAULT_POWER;

        //  Try with both..
        //  this.strafe(this.strafePower);  //  uses gyro;
        this.driver.strafe(strafePower);

        while (opModeIsActive() && !(colorSensor.isTargetBlue() || colorSensor.isTargetRed())) {
            telemetry.addData("ColorNotFound", "True");
            telemetry.update();
        }

        this.stopOpMode();

    }

    //  method helps to identify location of robot
    private GameQuadrant selectGameQuadrant() {

        // Wait until gamepad a or b is press for zone selection;

        GameQuadrant gq = null;

        boolean isLoadingZone = false;
        while (!gamepad1.a || !gamepad1.y) {
            // Tell the user what to press
            telemetry.addData("Instructions", "Press A if in LOADING ZONE; Press Y if in BUILDING ZONE");
            telemetry.update();
        }
        if (gamepad1.a) {
            isLoadingZone = true;
            telemetry.addData("Selected Zone", "LOADING ZONE");
        } else if (gamepad1.y) {
            isLoadingZone = false;
            telemetry.addData("Selected Zone", "BUILDING ZONE");
        }
        telemetry.update();

        sleep(1000);

        // Wait until gamepad press a or b is pressed for alliance selection;
        while (!gamepad1.a || !gamepad1.b) {
            telemetry.addData("Instructions", "Press B if in RED ALLIANCE; Press X if in BLUE ALLIANCE");
            telemetry.update();
        }
        if (gamepad1.b) {
            //  red alliance

            if (isLoadingZone) {
                gq = GameQuadrant.LOADING_RED;
            } else {
                gq = GameQuadrant.BUILDING_RED;
            }

        } else if (gamepad1.x) {
            //  blue alliance;

            if (isLoadingZone) {
                gq = GameQuadrant.LOADING_BLUE;
            } else {
                gq = GameQuadrant.BUILDING_BLUE;
            }
        }

        return gq;
    }
}