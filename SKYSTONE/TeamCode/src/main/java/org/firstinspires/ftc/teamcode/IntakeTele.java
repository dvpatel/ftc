package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.GamepadDriver;
import org.blueprint.ftc.core.IntakeSystem;

@TeleOp(name = "IntakeSystemTest", group = "Linear Opmode")
//  @Disabled
public class IntakeTele extends AbstractLinearOpMode {

    private IntakeSystem intakeSystem;

    @Override
    public void initOpMode() throws InterruptedException {
        //  Make sure Rosie is initialized ;
        this.initRosie();

        this.intakeSystem = rosie.getIntakeSystem();

        telemetry.addData("Mode", "init complete;  Running");
        telemetry.update();
    }

    @Override
    public void stopOpMode() {
        this.intakeSystem.stop();
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        this.initOpMode();

        //  Wait for start button ;
        this.waitToPressStart();

        while (opModeIsActive()) {
            this.intakeSystem.power(-gamepad1.left_stick_y);

            idle();
        }

        this.stopOpMode();
    }

}
