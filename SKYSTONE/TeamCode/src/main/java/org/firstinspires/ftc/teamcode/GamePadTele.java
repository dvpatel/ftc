package org.firstinspires.ftc.teamcode;


//  http://controls.coderedrobotics.com/programminglessons/11.html
//  https://drive.google.com/file/d/0B5ci5zMS_2kZUlRYaHZkMGNuZGc/view
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.Driver;
import org.blueprint.ftc.core.GamepadDriver;
import org.blueprint.ftc.core.IMUController;
import org.blueprint.ftc.core.IntakeSystem;
import org.blueprint.ftc.core.LiftSystem;
import org.blueprint.ftc.core.MotorControllerEx;
import org.blueprint.ftc.core.ServoController;

@TeleOp(name = "GamePadMode", group = "Tele")
//  @Disabled
public class GamePadTele extends AbstractLinearOpMode {

    private MotorControllerEx motor;
    private Driver driver;
    private IMUController imu;
    private ServoController shortArmServo;

    private IntakeSystem intakeSystem;
    private LiftSystem liftSystem;

    @Override
    public void initOpMode() throws InterruptedException {
        this.initRosie();

        this.shortArmServo = this.rosie.getShortArmServo();
        this.imu = this.rosie.getIMUController();

        this.intakeSystem = rosie.getIntakeSystem();

        this.liftSystem = this.rosie.getLiftSystem();
    }

    @Override
    public void stopOpMode() {

        this.intakeSystem.stop();

        this.stopDriving();
        //  this.servo.setPositionByDegrees(180);
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        this.initOpMode();

        telemetry.addData("Mode", "Init.Done");
        // telemetry.addData("ServoPos: ", servo.getPosition());
        telemetry.addData("Calibration Status:", this.imu.getCalibrationStatus());
        telemetry.update();

        //  Wait for start button ;
        this.waitToPressStart();

        telemetry.addData("Mode", "Started");

        GamepadDriver gpd = this.rosie.getGamepadDriver();
        while (opModeIsActive()) {

            //  Task 1:  Driving
            gpd.drive(gamepad1);

            //  Task 2:  ShortArmServo:  left_trigger, right_trigger
            double armPos = this.shortArmServo.triggerPosition(gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addData("ShortArm Position", "%.04f", armPos);

            //  Task 3:  IntakeSystem:  left, right bumpers
            //  gamepad left / right bumper to turn on and off intake system
            this.intakeSystem.autoMode(gamepad1);

            //  Arm system:  Gamepad2, left_stick_y; X, B; dpad_left, dpad_right
            //  0:  motor position, 1:  arm position, 2: slide servo position;
            double[] positions = this.liftSystem.autoMode(gamepad2);
            telemetry.addData("Motor, Arm, Slide Position", "%.04f, %.04f, %.0f",
                    positions[0], positions[1], positions[2]);

            telemetry.update();

            idle();
        }

        this.stopOpMode();
    }
}
