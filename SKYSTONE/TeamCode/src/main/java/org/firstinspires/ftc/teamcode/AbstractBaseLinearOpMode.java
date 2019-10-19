package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AbstractBaseLinearOpMode extends LinearOpMode {

    abstract void initRobot();

    abstract void stopRobot();

    protected GameBot rosie;


    protected void waitToPressStart() {
        // wait for start button.
        waitForStart();

        //  why is this needed?
        sleep(1000);
    }

    protected void initRosie() throws InterruptedException {
        this.rosie = new GameBot();
        this.rosie.init(hardwareMap);
    }

    public ColorSensorController getColorSensorController() {
        return this.rosie.getColorSensorController();
    }

}

