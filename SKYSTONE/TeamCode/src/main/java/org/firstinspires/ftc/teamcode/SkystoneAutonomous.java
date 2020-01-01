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

    private static final double DEFAULT_POWER = 1.0;
    private double strafePower;

    private static final int SLEEP_TIME = 100;

    //  Game quadrant;
    private GameQuadrant quadrant;

    private boolean quadrantSelected;

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

    private boolean isQuadrantSelected() {
        return this.quadrantSelected;
    }

    private void setQuadrantSelected() {
        this.quadrantSelected = true;
    }

    //  method helps to identify location of robot
    private GameQuadrant selectGameQuadrant() {

        // Wait until gamepad a or b is press for zone selection;
        GameQuadrant gq = null;
        boolean isLoadingZone = false;
        boolean isBuildingZone = false;

        while (!isQuadrantSelected()) {

            if (!isLoadingZone && !isBuildingZone) {

                telemetry.addData("Instructions", "Press A for LOADING ZONE; Press Y for BUILDING ZONE");

                if (gamepad1.a) {
                    isLoadingZone = true;
                } else if (gamepad1.y) {
                    isBuildingZone = true;
                }
            }

            if (isLoadingZone || isBuildingZone && gq == null) {

                telemetry.addData("Instructions", "Press X for BLUE ALLIANCE; Press B for RED ALLIANCE");

                if (gamepad1.b) {
                    //  red alliance

                    if (isLoadingZone) {
                        gq = GameQuadrant.LOADING_RED;
                    } else if (isBuildingZone) {
                        gq = GameQuadrant.BUILDING_RED;
                    }

                } else if (gamepad1.x) {
                    //  blue alliance;

                    if (isLoadingZone) {
                        gq = GameQuadrant.LOADING_BLUE;
                    } else if (isBuildingZone) {
                        gq = GameQuadrant.BUILDING_BLUE;
                    }
                }
            }

            telemetry.update();

            if (gq != null) {
                this.setQuadrantSelected();
            }
        }

        return gq;
    }
}