package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseLinearOpMode extends AbstractBaseLinearOpMode {

    abstract void initRobot() ;
    abstract void runRobot() ;
    abstract void stopRobot() ;

    // called when init button is  pressed.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    @Override
    public void runOpMode() throws InterruptedException
    {
        //  Put common init logic here
        this.initRosie();

        this.initRobot();

        this.waitToPressStart();

        while (opModeIsActive()) {
            this.runRobot();
        }

        this.stopRobot();
    }

}
