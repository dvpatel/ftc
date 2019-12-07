// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses PID controller to drive in a straight line when not
// avoiding an obstacle.
//
// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "GyroTurnPID", group = "Auto")
//@Disabled
public class GyroTurnPid extends AbstractLinearOpMode {
    private MotorControllerEx motor;
    private Driver driver;
    private IMUController imu;

    double power = 0.50;
    int degrees = 180;

    @Override
    void initOpMode() throws InterruptedException {
        this.initRosie();

        this.motor = this.rosie.getMotorPID();  //  Defaut power ;
        this.driver = this.rosie.getDriver();
        this.imu = this.rosie.getIMUController();
        this.imu.resetAngle();

        telemetry.addData("Calibration status", this.imu.getCalibrationStatus());
        telemetry.addData("IMU Angle", this.imu.getAngle());
        telemetry.addData("Status:  ", "ready");

        telemetry.update();
    }

    @Override
    void stopOpMode() {
        // turn the motors off.
        driver.stop();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        //  this.imu.resetAngle();
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        //  Put common init logic here
        this.initOpMode();

        // wait for start button.
        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();

        do {
            this.turn(degrees, power);
        } while (opModeIsActive() && !this.motor.angleOnTarget());

        telemetry.addData("IMU Angle", this.imu.getAngle());
        telemetry.update();

        sleep(5000);
        this.stopOpMode();
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle() {
        double angle = this.imu.getAngle();
        telemetry.addData("Imu.Angle", angle);
        telemetry.update();
        return angle;
    }
}