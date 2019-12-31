package org.firstinspires.ftc.teamcode;


//  http://controls.coderedrobotics.com/programminglessons/11.html
//  https://drive.google.com/file/d/0B5ci5zMS_2kZUlRYaHZkMGNuZGc/view
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.Driver;
import org.blueprint.ftc.core.GamepadDriver;
import org.blueprint.ftc.core.IMUController;
import org.blueprint.ftc.core.IntakeSystem;
import org.blueprint.ftc.core.MotorControllerEx;
import org.blueprint.ftc.core.ServoController;

@TeleOp(name = "GamePadDrive", group = "Tele")
//  @Disabled
public class GamePadTele extends AbstractLinearOpMode {

    private MotorControllerEx motor;
    private Driver driver;
    private IMUController imu;
    private ServoController servo;

    private IntakeSystem intakeSystem;

    //  private double rotation ;
    private boolean aButton, bButton, touched;

    @Override
    public void initOpMode() throws InterruptedException {
        this.initRosie();

        this.servo = this.rosie.getShortArmServo();
        this.imu = this.rosie.getIMUController();

        this.intakeSystem = rosie.getIntakeSystem();

        //  Enable PID Controller to track state
        //  this.motor.enablePID();
        //this.motor.enableDrivePID(power);
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
            double[] p = gpd.calculatePowerDifferential(gamepad1);
            gpd.drive(gamepad1);
            telemetry.addData("PowerDifferential:  ", p[0] + ", " + p[1] + ", " + p[2] + ", " + p[3]);

            //  Task 2:  ShortArmServo
            this.servo.triggerPosition(gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addData("LeftTrigger", gamepad1.left_trigger);
            telemetry.addData("RightTrigger", gamepad1.right_trigger);
            telemetry.addData("ServoPos", this.servo.getPosition());

            //  Task 3:  IntakeSystem
            //  gamepad left / right bumper to turn on and off intake system
            this.intakeSystem.autoMode(gamepad1);

            //  Telemetry print for debugging;
            telemetry.update();

            idle();
        }

        this.stopOpMode();
    }
}
