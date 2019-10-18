package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public abstract class BaseLinearOpMode extends LinearOpMode {

    abstract void initRobot() ;
    abstract void runRobot() ;
    abstract void stopRobot() ;

    protected void waitToPressStart() {
        // wait for start button.
        waitForStart();

        //  why is this needed?
        sleep(1000);
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        this.initRobot();

        this.waitToPressStart();

        while (opModeIsActive()) {
            this.runRobot();
        }

        this.stopRobot();
    }

}
