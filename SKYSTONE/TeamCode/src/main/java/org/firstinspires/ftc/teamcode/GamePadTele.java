package org.firstinspires.ftc.teamcode;


//  http://controls.coderedrobotics.com/programminglessons/11.html

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GamePad", group = "Tele")
//  @Disabled
public class GamePadTele extends AbstractLinearOpMode {

    private MotorControllerEx motor;
    private Driver driver;
    private IMUController imu;

    //  private double rotation ;
    private boolean aButton, bButton, touched;

    private double power;

    @Override
    void initOpMode() throws InterruptedException {

        telemetry.addData("Mode", "init Rosie");
        telemetry.update();

        this.initRosie();

        //  Enable PID Controller to track state
        //  this.motor.enablePID();
        //this.motor.enableDrivePID(power);

        telemetry.addData("Mode", "init complete;  Running");
        telemetry.update();
    }

    @Override
    void stopOpMode() {
        this.stopDriving();
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Mode", "Start.INit");
        telemetry.update();

        this.initOpMode();

        telemetry.addData("Mode", "InitMode.Done");


        //  Wait for start button ;
        this.waitToPressStart();

        telemetry.addData("Mode", "Started");

        GamepadDriver gpd = this.rosie.getGamepadDriver();
        while (opModeIsActive()) {

            //  Uncomment when real motors attached ; Direction correct?
            double[] p = gpd.calculatePowerDifferential(gamepad1);
            gpd.drive(gamepad1);

            //  Controls direction
            telemetry.addData("LeftF", p[0]);
            telemetry.addData("RightF", p[1]);
            telemetry.addData("LeftB", p[2]);
            telemetry.addData("RightB", p[3]);
            telemetry.update();

            idle();
        }

        this.stopOpMode();
    }
}
