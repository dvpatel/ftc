package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DriveToLine", group = "Auto")
public class DriveToLIne extends AbstractLinearOpMode {


    private Driver driver;
    private ColorSensorController colorSensor;

    private String zone;
    private int quadrant;

    private int powerToStrafe;
    private int distanceToQuadrant;

    @Override
    void initOpMode() throws InterruptedException {

        this.driver = this.rosie.getDriver();
        this.colorSensor = this.rosie.getColorSensorController();


        // Tell the user what to press
        telemetry.addData("Instructions", "Press A if in loading zone; Press B if in building zone");
        telemetry.update();

        // Wait until gamepad press
        while (!gamepad1.a || !gamepad1.b) {}
        // choose if left or right of line
        if (gamepad1.a) {
            this.zone = "loading zone";
            this.powerToStrafe = 10;
        } else if (gamepad1.b) {
            this.zone = "building zone";
            this.powerToStrafe = -10;
        } else {
            telemetry.addData("Error", "Selecting zones failed!");
            telemetry.update();
            stopOpMode();
        }

        // Tell the user what to press
        telemetry.addData("Instructions", "Press X if requesting first quadrant; Press Y if requesting second quadrant");
        telemetry.update();

        // Wait until gamepad press
        while (!gamepad1.x || !gamepad1.y) {}

        if (gamepad1.x) {
            this.quadrant = 1;
            this.distanceToQuadrant = 0;
        } else if (gamepad1.y) {
            this.quadrant = 2;
            this.distanceToQuadrant = Constants.TILE_LENGTH;
        } else {
            telemetry.addData("Error", "Selecting quadrants failed!");
            telemetry.update();
            stopOpMode();
        }


        }

    @Override
    public void runOpMode() throws InterruptedException {

        this.driver.setTargetPosition(quadrant);

        do {
            this.driver.strafe(powerToStrafe);
        } while (!this.colorSensor.isTargetRed() || !this.colorSensor.isTargetBlue());

        this.driver.strafe(0);

    }

    @Override
    void stopOpMode() {
        driver.stop();
    }
}
