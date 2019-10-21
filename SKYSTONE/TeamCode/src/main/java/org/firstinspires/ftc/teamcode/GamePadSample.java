package org.firstinspires.ftc.teamcode;


//  https://drive.google.com/file/d/0B5ci5zMS_2kZUlRYaHZkMGNuZGc/view

public class GamePadSample extends AbstractLinearOpMode {

    private MotorControllerEx motor;
    private Driver driver;
    private IMUController imu;

    //  private double rotation ;
    private boolean aButton, bButton, touched;

    private double power;

    @Override
    void initOpMode() throws InterruptedException {

        this.initRosie();

        //  Set default power ;
        this.power = this.normalizePower(0.3);

        this.imu = this.rosie.getIMUController();
        this.motor = this.rosie.getMotorPID();
        this.driver = this.rosie.getDriver();

        //  Enable PID Controller to track state
        //  this.motor.enablePID();
        //this.motor.enableDrivePID(power);

        telemetry.addData("Mode", "init complete;  Running");
        telemetry.update();
    }

    @Override
    void stopOpMode() {
        driver.stop();
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        this.initOpMode();

        //  Wait for start button ;
        this.waitToPressStart();


        //  Pushing the right joystick on its y-axis gives a y-value
        //   (+ = forward, - = backward)
        //  Pushing the right joystick on its x-axis gives a x-value
        //   (+ = right, - = left)
        //  Pushing the left joystick on its x-axis gives a r-value
        //   (+ = clockwise, - = counter-clockwise)
        //  Left joystick y-axis unused


        //  Front_Left_Power = + x + y + (k*r)   what is k, r?
        //  Front_Right_Power = - x + y - (k*r)
        //  Back_Left_Power = - x + y + (k*r)
        //  Back_Right_Power = + x + y - (k*r)

        double rightY = 0;
        double rightX = 0;
        double leftX = 0;

        while (opModeIsActive()) {

            rightY = this.normalizePower(gamepad1.right_stick_y + rightY);
            rightX = this.normalizePower(gamepad1.right_stick_x + rightX);
            leftX = this.normalizePower(gamepad1.left_stick_x + leftX);

            telemetry.addData("RightY", rightY);
            telemetry.addData("RightX", rightX);
            telemetry.addData("LeftX", leftX);
            telemetry.update();

            //  this.drive(rightY) ;
            //  this.strafe(rightX);
            //  this.rotate(leftX);

            idle();
        }

        this.stopOpMode();
    }
}
