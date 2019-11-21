package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "ServoTest", group = "Tele")
// @Disabled
public class ServoTele extends AbstractLinearOpMode {

    private ServoController servo;

    @Override
    void initOpMode() throws InterruptedException {
        telemetry.addData("Mode", "init Rosie");
        telemetry.update();

        this.initRosie();

        //  Anish:  Start at 180 or down position;
        this.servo = this.rosie.getShortArmServo();
        this.servo.setPositionByDegrees(180);

        telemetry.addData("Mode", "init complete;  Running");
        telemetry.update();
    }

    @Override
    void stopOpMode() {
        this.servo.setPositionByDegrees(0);
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        this.initOpMode();

        telemetry.addData("ServoPos: ", this.servo.getPosition());
        telemetry.update();

        //  Wait for start button ;
        this.waitToPressStart();

        telemetry.addData("Mode", "Started");
        telemetry.update();

        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            //  "Y" button --> 0 degrees;  X or B button 90 degrees; and A for 180 degrees;
            if (gamepad1.y) {
                //  Drop on stop at 10 degrees ;
                this.servo.setPositionByDegrees(10);
            } else if (gamepad1.x || gamepad1.b) {
                this.servo.setPositionByDegrees(90);
            } else if (gamepad1.a) {
                this.servo.setPositionByDegrees(180);
            }

            telemetry.addData("ServoPos", this.servo.getPosition());
            telemetry.update();
        }

        this.stopOpMode();
    }
}
