package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.ServoController;


@TeleOp(name = "ShortArmServoTele", group = "Tele")
@Disabled
public class ServoTele extends AbstractLinearOpMode {

    private ServoController servo;

    @Override
    public void initOpMode() throws InterruptedException {
        telemetry.addData("Mode", "init Rosie");
        telemetry.update();

        this.initRosie();

        //  Anish:  Start at 180 or down position;
        this.servo = this.rosie.getShortArmServo();

        telemetry.addData("Mode", "init complete;  Running");
        telemetry.update();
    }

    @Override
    public void stopOpMode() {
        this.servo.setPositionByDegrees(180);
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        this.initOpMode();

        telemetry.addData("ServoPos: ", servo.getPosition());
        telemetry.update();

        //  Wait for start button ;
        this.waitToPressStart();

        telemetry.addData("Mode", "Started");
        telemetry.update();

        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            this.servo.triggerPosition(gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addData("ServoPos", this.servo.getPosition());
            telemetry.update();

            idle();
        }

        this.stopOpMode();
    }
}
