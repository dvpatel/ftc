package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.Constants;
import org.blueprint.ftc.core.Driver;
import org.blueprint.ftc.core.FoundationSystem;
import org.blueprint.ftc.core.GamepadDriver;
import org.blueprint.ftc.core.IMUController;
import org.blueprint.ftc.core.IntakeSystem;
import org.blueprint.ftc.core.LiftSystem;
import org.blueprint.ftc.core.MotorControllerEx;


/**
 * Gamepad1 Controls:
 *
 *  left_stick_y:   drive forward / reverse
 *  left_stick_x:   strafe left / right
 *  right_stick_x:  turn left / right
 *  X:  Put robot in forward, regular drive mode
 *  B:  Put robot in reverse drive mode
 *  left_trigger:  Foundation system up
 *  right_trigger:  Foundation system down
 *  left_bumper:  Intake system reverse
 *  right_bumper:  Intake system forward
 *  A:  Intake system off
 *
 *
 * Gamepad2 Controls:
 *
 *  left_stick_y:   Drive lift system up and down
 *  right_stick_y:  Drive lift system arm up and down
 *  Y:  Lift system automode to pickup block from storage
 *  A:  Lift system automode to go back to base
 *  dpad_left:   Open lift system gripper
 *  dpad_right:  Close lift system gripper
 *
 */

@TeleOp(name = "GamepadTele")
//  @Disabled
public class SkystoneTele extends AbstractLinearOpMode {

    private MotorControllerEx motor;
    private IMUController imu;

    private Driver driver;
    private FoundationSystem foundationSystem;
    private IntakeSystem intakeSystem;
    private LiftSystem liftSystem;
    private GamepadDriver gpd;

    @Override
    public void initOpMode() throws InterruptedException {
        this.initRosie();

        this.foundationSystem = this.rosie.getFoundationSystem();
        this.imu = this.rosie.getIMUController();
        this.imu.resetAngle();

        this.intakeSystem = rosie.getIntakeSystem();

        this.liftSystem = this.rosie.getLiftSystem();
        this.liftSystem.setLinearOpMode(this);

        this.gpd = this.rosie.getGamepadDriver();
    }

    //  Post start init logic;  robot can only expand after start of game or after press start
    public void postStartSetup() throws InterruptedException {

        this.intakeSystem.setIntakeServosInitPosition();

    }

    @Override
    public void stopOpMode() {

        this.stop();

        this.stopDriving();
        this.intakeSystem.stop();
        this.foundationSystem.triggerUp(true);
        this.liftSystem.moveBackSlide();
    }

    private void addGamepadTelemetry() {
        telemetry.addData("Gamepad1.LeftStickY", gamepad1.left_stick_y);
        telemetry.addData("Gamepad1.LeftStickX", gamepad1.left_stick_x);
        telemetry.addData("Gamepad1.RightStickX", gamepad1.right_stick_x);
        telemetry.addData("Gamepad1.B", gamepad1.b);
        telemetry.addData("Gamepad1.X", gamepad1.x);
        telemetry.addData("Gamepad1.LeftTrigger", gamepad1.left_trigger);
        telemetry.addData("Gamepad1.RightTrigger", gamepad1.right_trigger);
        telemetry.addData("Gamepad1.LeftBumper", gamepad1.left_bumper);
        telemetry.addData("Gamepad1.RightBumper", gamepad1.right_bumper);
        telemetry.addData("Gamepad1.A", gamepad1.a);
        telemetry.addData("Gamepad2.LeftStickY", gamepad2.left_stick_y);
        telemetry.addData("Gamepad2.RightStickY", gamepad2.right_stick_y);
        telemetry.addData("Gamepad2.Y", gamepad2.y);
        telemetry.addData("Gamepad2.A", gamepad2.a);
        telemetry.addData("Gamepad2.DpadLeft", gamepad2.dpad_left);
        telemetry.addData("Gamepad2.DpadRight", gamepad2.dpad_right);
    }

    class Gamepad1Thread implements Runnable {
        @Override
        public void run() {
            while(opModeIsActive()) {
                telemetry.addData("Gamepad Action:  ", "1");

                //  Change driving direction
                gpd.putInReverse(gamepad1.b);
                gpd.putInDrive(gamepad1.x);
                //  telemetry.addData("Driving Reverse", gpd.getDrivingDirection());

                foundationSystem.triggerUp(gamepad1.left_trigger > 0.25);
                foundationSystem.triggerDown(gamepad1.right_trigger > 0.25);

                intakeSystem.autoMode(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.a);

                //  forward, sideways, turn;
                gpd.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

                idle();
            }
        }
    }

    class Gamepad2Thread implements Runnable {
        @Override
        public void run() {
            while(opModeIsActive()) {
                telemetry.addData("Gamepad Action:  ", "2");

                //  Automode to pickup block from storage
                liftSystem.pickup(gamepad2.y);

                //  put liftsystem back to base
                liftSystem.backToBase(gamepad2.a);

                liftSystem.getLinearArmServo().openGripper(gamepad2.dpad_left);
                liftSystem.getLinearArmServo().closeGripper(gamepad2.dpad_right);

                //  Up / Down Linear
                liftSystem.drive(-gamepad2.left_stick_y);

                //  Up / Down slide
                liftSystem.positionSlideServo(-gamepad2.right_stick_y);

                idle();
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        this.initOpMode();

        Thread t1 = new Thread(new Gamepad1Thread());
        Thread t2 = new Thread(new Gamepad2Thread());
        t1.start();
        t2.start();

        this.waitToPressStart();

        this.postStartSetup();

        while (opModeIsActive()) {
            //  this.addGamepadTelemetry();
            telemetry.update();
        }

        this.stopOpMode();
    }

    public void runOpModeGood() throws InterruptedException {

        this.initOpMode();
        this.waitToPressStart();

        this.postStartSetup();

        while (opModeIsActive()) {

            //  this.addGamepadTelemetry();
            telemetry.addData("Driving Reverse", gpd.getDrivingDirection());

            //  Change driving direction
            this.gpd.putInReverse(gamepad1.b);
            this.gpd.putInDrive(gamepad1.x);

            this.foundationSystem.triggerUp(gamepad1.left_trigger > 0.25);
            this.foundationSystem.triggerDown(gamepad1.right_trigger > 0.25);

            this.intakeSystem.autoMode(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.a);

            //  Automode to pickup block from storage
            this.liftSystem.pickup(gamepad2.y);

            //  put liftsystem back to base
            this.liftSystem.backToBase(gamepad2.a);

            this.liftSystem.getLinearArmServo().openGripper(gamepad2.dpad_left);
            this.liftSystem.getLinearArmServo().closeGripper(gamepad2.dpad_right);

            //  forward, sideways, turn;
            this.gpd.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            //  Up / Down Linear
            this.liftSystem.drive(-gamepad2.left_stick_y);

            //  Up / Down slide
            this.liftSystem.positionSlideServo(-gamepad2.right_stick_y);

            telemetry.update();
        }

        this.stopOpMode();
    }
}
