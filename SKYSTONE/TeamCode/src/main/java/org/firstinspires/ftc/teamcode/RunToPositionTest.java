package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.blueprint.ftc.core.Constants;

@TeleOp
@Disabled
public class RunToPositionTest extends LinearOpMode {

    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;

    @Override
    public void runOpMode() {

        leftFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get(Constants.LEFT_FRONT_MOTOR_NAME);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get(Constants.RIGHT_FRONT_MOTOR_NAME);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackMotor = (DcMotorEx) hardwareMap.dcMotor.get(Constants.LEFT_BACK_MOTOR_NAME);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightBackMotor = (DcMotorEx) hardwareMap.dcMotor.get(Constants.RIGHT_BACK_MOTOR_NAME);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  Set PID;
        this.setVelocityPID();    //  When using RUN_USING_ENCODER;
        this.setPositionalPID();  //  if using Run_To_Position;

        waitForStart();

        // Set the motor's target position to 300 ticks
        int ticks = (int) (Constants.MOTOR_TICK_COUNT * 2);
        leftFrontMotor.setTargetPosition(ticks);
        rightFrontMotor.setTargetPosition(ticks);
        leftBackMotor.setTargetPosition(ticks);
        rightBackMotor.setTargetPosition(ticks);

        // Switch to RUN_TO_POSITION mode
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 200 ticks per second
        leftFrontMotor.setVelocity(Constants.MOTOR_MAX_VELOCITY);
        rightFrontMotor.setVelocity(Constants.MOTOR_MAX_VELOCITY);
        leftBackMotor.setVelocity(Constants.MOTOR_MAX_VELOCITY);
        rightBackMotor.setVelocity(Constants.MOTOR_MAX_VELOCITY);

        // While the Op Mode is running, show the motor's status via telemetry
        while (opModeIsActive() && this.isBusy()) {

            double maxV[] = this.getVelocity();
            telemetry.addData("Velocity", "%.04f, %.04f, %.0f, %.0f",
                    maxV[0], maxV[1], maxV[2], maxV[3]);

            int currentPosition[] = this.getCurrentPosition();
            telemetry.addData("CurrentPosition", "%d, %d, %d, %d",
                    currentPosition[0], currentPosition[1], currentPosition[2], currentPosition[3]);

            telemetry.addData("is at target", !this.isBusy());
            telemetry.update();
        }

        leftFrontMotor.setVelocity(0);
        rightFrontMotor.setVelocity(0);
        leftBackMotor.setVelocity(0);
        rightBackMotor.setVelocity(0);
    }

    //  Must set properly;
    private void setVelocityPID() {
        //  Get values from MaxVelocityTest;
        this.leftFrontMotor.setVelocityPIDFCoefficients(Constants.PID_DRIVE_KP, Constants.PID_DRIVE_KI, Constants.PID_DRIVE_KD, Constants.PID_DRIVE_KF);
        this.rightFrontMotor.setVelocityPIDFCoefficients(Constants.PID_DRIVE_KP, Constants.PID_DRIVE_KI, Constants.PID_DRIVE_KD, Constants.PID_DRIVE_KF);
        this.leftBackMotor.setVelocityPIDFCoefficients(Constants.PID_DRIVE_KP, Constants.PID_DRIVE_KI, Constants.PID_DRIVE_KD, Constants.PID_DRIVE_KF);
        this.rightBackMotor.setVelocityPIDFCoefficients(Constants.PID_DRIVE_KP, Constants.PID_DRIVE_KI, Constants.PID_DRIVE_KD, Constants.PID_DRIVE_KF);
    }

    private void setPositionalPID() {
        this.leftFrontMotor.setPositionPIDFCoefficients(Constants.POSITIONAL_DRIVE_KP);
        this.rightFrontMotor.setPositionPIDFCoefficients(Constants.POSITIONAL_DRIVE_KP);
        this.leftBackMotor.setPositionPIDFCoefficients(Constants.POSITIONAL_DRIVE_KP);
        this.rightBackMotor.setPositionPIDFCoefficients(Constants.POSITIONAL_DRIVE_KP);
    }


    public boolean isBusy() {
        return this.leftFrontMotor.isBusy() &&
                this.rightFrontMotor.isBusy() &&
                this.leftBackMotor.isBusy() &&
                this.rightBackMotor.isBusy();
    }

    private double[] getVelocity() {
        double[] v = {
                leftFrontMotor.getVelocity(),
                rightFrontMotor.getVelocity(),
                leftBackMotor.getVelocity(),
                rightBackMotor.getVelocity()};
        return v;
    }

    private int[] getCurrentPosition() {
        int[] v = {
                leftFrontMotor.getCurrentPosition(),
                rightFrontMotor.getCurrentPosition(),
                leftBackMotor.getCurrentPosition(),
                rightBackMotor.getCurrentPosition()};
        return v;
    }
}
