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

        telemetry.addData("Mode:  ", "init complete;  Running");
        telemetry.update();
    }

    @Override
    protected void stopOpMode() {
        driver.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //  Put common init logic here
        this.initOpMode();

        //  Select quadrant;
        this.selectGameQuadrant();
        telemetry.addData("Selected Zone:  ", quadrant.toString());
        telemetry.addData("Power Direction (- left, + right):  ", GameQuadrant.direction(quadrant));
        telemetry.update();

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
        this.strafe(this.strafePower);
        while (opModeIsActive() && !(colorSensor.isTargetBlue() || colorSensor.isTargetRed())) {
            telemetry.addData("ColorNotFound", "True");
            telemetry.update();
        }

        this.stopOpMode();

    }

    //  method helps to identify location of robot
    private void selectGameQuadrant() {

        // Wait until gamepad a or b is press for zone selection;

        boolean isLoadingZone = false;
        while (!gamepad1.a || !gamepad1.b) {
            // Tell the user what to press
            telemetry.addData("Instructions", "Press A if in LOADING ZONE; Press B if in building zone");
            telemetry.update();
        }
        if (gamepad1.a) {
            isLoadingZone = true;
        } else if (gamepad1.b) {
            isLoadingZone = false;
        }

        // Wait until gamepad press a or b is pressed for alliance selection;
        while (!gamepad1.a || !gamepad1.b) {
            telemetry.addData("Instructions", "Press A if in RED ALLIANCE; Press B if in BLUE ALLIANCE");
            telemetry.update();
        }
        if (gamepad1.a) {
            //  red alliance

            if (isLoadingZone) {
                this.quadrant = GameQuadrant.LOADING_RED;
            } else {
                this.quadrant = GameQuadrant.BUILDING_RED;
            }

        } else if (gamepad1.b) {
            //  blue alliance;

            if (isLoadingZone) {
                this.quadrant = GameQuadrant.LOADING_BLUE;
            } else {
                this.quadrant = GameQuadrant.BUILDING_BLUE;
            }
        }

    }
}