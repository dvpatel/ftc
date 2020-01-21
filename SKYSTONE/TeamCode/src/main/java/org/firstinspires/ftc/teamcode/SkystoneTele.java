package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.blueprint.ftc.core.AbstractLinearOpMode;
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
        this.stopDriving();
        this.intakeSystem.stop();
        this.foundationSystem.triggerUp();
        this.liftSystem.moveBackSlide();
    }


    @Override
    public void runOpMode() throws InterruptedException {

        this.initOpMode();
        this.waitToPressStart();

        this.postStartSetup();

        while (opModeIsActive()) {

            telemetry.addData("Driving Reverse", gpd.getDrivingDirection());
            if (gamepad1.b) {

                //  Change driving direction
                this.gpd.putInReverse();

            } else if (gamepad1.x) {

                this.gpd.putInDrive();

            } else if (gamepad1.left_trigger > 0.25) {

                this.foundationSystem.triggerUp();

            } else if (gamepad1.right_trigger > 0.25) {

                this.foundationSystem.triggerDown();

            } else if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.a) {

                this.intakeSystem.autoMode(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.a);

            } else if (gamepad2.y) {  //  gamepad2

                this.liftSystem.pickup();  //  Automode to pickup block from storage

            } else if (gamepad2.a) {

                this.liftSystem.backToBase(); //  put liftsystem back to base

            } else if (gamepad2.dpad_left) {

                this.liftSystem.getLinearArmServo().openGripper();

            } else if (gamepad2.dpad_right) {

                this.liftSystem.getLinearArmServo().closeGripper();

            }

            telemetry.update();

            //  forward, sideways, turn;
            this.gpd.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            //  Up / Down Linear
            this.liftSystem.drive(-gamepad2.left_stick_y);

            //  Up / Down slide
            this.liftSystem.positionSlideServo(-gamepad2.right_stick_y);

            idle();
        }

        this.stopOpMode();
    }

}
