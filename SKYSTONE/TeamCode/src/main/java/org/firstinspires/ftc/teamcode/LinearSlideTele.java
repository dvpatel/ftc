package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LinearSlideTest", group = "Linear Opmode")
//@Disabled
public class LinearSlideTele extends AbstractLinearOpMode {

    private SimpleMotor linearSlideMotor;
    private ServoController linearSlideServo;
    private ServoController linearArmServo;

    double power = this.normalizePower(0.50);

    @Override
    public void initOpMode() throws InterruptedException {

        telemetry.addData("Mode", "init Rosie");
        telemetry.update();

        //  Make sure Rosie is initialized ;
        this.initRosie();

        this.linearSlideMotor = this.rosie.getLinearSlideMotor();
        this.linearSlideServo = this.rosie.getLinearSlideServo();
        this.linearArmServo = this.rosie.getLinearArmServo();

        telemetry.addData("Mode", "init complete;  Running");
        telemetry.update();
    }

    @Override
    void stopOpMode() {
        this.linearSlideMotor.stop();
        this.linearArmServo.setPositionByDegrees(180);
        this.linearSlideServo.setPositionByDegrees(180);
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        this.initOpMode();

        GamepadDriver gpd = this.rosie.getGamepadDriver();

        //  Wait for start button ;
        this.waitToPressStart();

        //  Tells motor to run to target using position and power ;  Make sure t reset encoder when done!
        //  this.simpleMotor.setTargetPosition(2500);

        // motorsBusy / current position only works when using encoder wires;
        while (opModeIsActive()) {
            this.linearSlideMotor.power(-gamepad1.left_stick_y);

            telemetry.addData("CurrentPosition", this.linearSlideMotor.getCurrentPosition());
            telemetry.update();
        }

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 10) {
            telemetry.addData("CurrentPosition", this.linearSlideMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        this.stopOpMode();
    }


}
