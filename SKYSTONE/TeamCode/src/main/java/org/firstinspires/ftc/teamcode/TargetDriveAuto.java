package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TargetDrive", group = "Auto")
//  @Disabled
public class TargetDriveAuto extends AbstractLinearOpMode {

    private MotorControllerEx motor;
    private Driver driver;
    private IMUController imu;

    @Override
    void initOpMode() throws InterruptedException {

        this.initRosie();

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

    @Override
    public void runOpMode() throws InterruptedException {
        //  Put common init logic here
        this.initOpMode();

        //  Activate opmode
        this.waitToPressStart();

        //  make sure power is between -1 and 1 ;
        double power = this.normalizePower(0.3);

        telemetry.addData("Forward:  ", "forward");
        telemetry.update();
        this.drive(6, power);
        sleep(1000);

        telemetry.addData("GyroStrafe:  ", "strafe left");
        telemetry.update();
        this.strafe(6, power);
        sleep(5000);

        //  drive backward 12 inches ;  Both power and distance needs to be negative
        telemetry.addData("GyroDrive:  ", "back 6 inches");
        telemetry.update();
        this.drive(-6, -power);
        sleep(1000);

        //  Strafe 12 inches right ; both values must be negative ;
        telemetry.addData("GyroStrafe:  ", "strafe 6 inches right");
        telemetry.update();
        this.strafe(-6, -power);

        //  Turn 90 degrees to the right
        telemetry.addData("GyroTurn:  right ", "90 degrees");
        telemetry.update();
        this.turn(90, power);
        telemetry.addData("GyroTurn:  Done, ", imu.getAngle());
        telemetry.update();
        sleep(5000);

        //  Turn 90 degrees left;  note degrees direction and power
        telemetry.addData("GyroTurn: left  ", "90 degrees??");
        telemetry.update();
        this.turn(-90, power);
        telemetry.addData("GyroTurn:  Done, ", imu.getAngle());
        telemetry.update();

    }
}
