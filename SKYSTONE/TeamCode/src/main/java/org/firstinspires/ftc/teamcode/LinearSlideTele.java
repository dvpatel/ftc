package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.GamepadDriver;
import org.blueprint.ftc.core.ServoController;
import org.blueprint.ftc.core.SimpleMotor;

@TeleOp(name = "LinearSlideTest", group = "Linear Opmode")
//  @Disabled
public class LinearSlideTele extends AbstractLinearOpMode {

    private SimpleMotor linearSlideMotor;
    private ServoController linearSlideServo;
    private ServoController linearArmServo;

    private static final int DISTANCE_IN_INCHES = 24;
    private static final int SLEEP_TIME = 500;
    private static final double POWER_LEVEL = 0.90;

    @Override
    public void initOpMode() throws InterruptedException {
        //  Make sure Rosie is initialized ;
        this.initRosie();

        this.linearSlideMotor = this.rosie.getLinearSlideMotor();
        this.linearSlideServo = this.rosie.getLinearSlideServo();
        //  this.linearArmServo = this.rosie.getLinearArmServo();

        telemetry.addData("Mode", "init complete;  Running");
        telemetry.update();
    }

    @Override
    public void stopOpMode() {
        this.linearSlideMotor.stop();
        this.linearSlideServo.setPositionByDegrees(0);
        //  this.linearArmServo.setPositionByDegrees(180);
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

        //  make sure power is between -1 and 1 ;
        telemetry.addData("LinearSlide:  ", "forward");
        telemetry.update();
        this.linearSlideDriveForward(DISTANCE_IN_INCHES, POWER_LEVEL);
        sleep(SLEEP_TIME);

        this.linearSlideServo.setPositionByDegrees(0);
        telemetry.addData("LinearSlideServoStart:  ", this.linearSlideServo.getPosition());
        telemetry.update();
        sleep(SLEEP_TIME);
        this.linearSlideServo.setPositionByDegrees(180);
        telemetry.addData("LinearSlideServoMid:  ", this.linearSlideServo.getPosition());
        telemetry.update();
        sleep(SLEEP_TIME);
        this.linearSlideServo.setPositionByDegrees(0);
        telemetry.addData("LinearSlideServoEnd:  ", this.linearSlideServo.getPosition());
        telemetry.update();
        sleep(SLEEP_TIME);

        telemetry.addData("LinearSlide:  ", "reverse");
        telemetry.update();
        this.linearSlideDriveReverse(DISTANCE_IN_INCHES, POWER_LEVEL);
        sleep(SLEEP_TIME);


        // motorsBusy / current position only works when using encoder wires;
        while (opModeIsActive()) {
            this.linearSlideMotor.drive(-gamepad1.left_stick_y);

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
