package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.blueprint.ftc.core.Constants;

import java.util.ArrayList;
import java.util.List;

//  NOTE:  TEST WITH ROBOT LOAD!!!  DON'T PUT ON SKYSTONE BLOCK

@TeleOp(name = "MaxVelocityTest", group = "Linear Opmode")
public class MaxVelocityTest extends LinearOpMode {

    private static final double DISTANCE = 12; //  inches

    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;

    int[] currentPosition = { 0, 0, 0, 0};

    double[] currentVelocity = { 0, 0, 0, 0};
    double[] maxVelocity = { 2900, 2900, 2900, 2900 };

    double[] velocity = { 0, 0, 0, 0};

    double[] P = { 1.1299, 1.1299, 1.1000, 1.1000};
    double[] I = { 0.11299, 0.11299, 0.11, 0.11 };
    double[] D = { 0, 0, 0, 0};
    double[] F = { 11.2990, 11.2990, 11.0, 11.0 };


    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.setupMotors();

        int distanceInTicks = this.toTicks(DISTANCE);
        telemetry.addData("Distance Ticks:", distanceInTicks);

        waitForStart();

        //  Run this method to determine PIDF values;  Make sure robot is running with load!
        //  this.calculatePIDF(distanceInTicks);
        //  Update Constants file:  PID, MAX_Velocity;

        //  Run this to validate PIDF values;  Besure to update maxVelocity and PIDF variables
        this.testWithPIDFValues(distanceInTicks);

        this.stop();
    }

    private void testWithPIDFValues(int distanceInTicks) {

        //  Verify PID values;
        this.setToEncoderMode();

        //  Set at 50% max velocity;
        for (int i = 0; i < 4; i++) {
            velocity[i] = 0.15 * maxVelocity[i];
        }
        this.setVelocity(velocity);

        while (opModeIsActive() && !distanceReached(distanceInTicks)) {

            currentVelocity = this.getVelocity();
            currentPosition = this.getCurrentPosition();

            //  Returns ticks per second;
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("Specified Velocity", "%.04f, %.04f, %.0f, %.0f",
                    velocity[0], velocity[1], velocity[2], velocity[3]);

            telemetry.addData("Current Velocity", "%.04f, %.04f, %.0f, %.0f",
                    currentVelocity[0], currentVelocity[1], currentVelocity[2], currentVelocity[3]);

            telemetry.addData("Current Position", "%d, %d, %d, %d",
                    currentPosition[0], currentPosition[1], currentPosition[2], currentPosition[3]);

            telemetry.update();
        }
        this.stopPower();


    }

    private List<double[]> cv = new ArrayList<double[]>();

    private void calculatePIDF(int distanceInTicks) {

        double[] maxV = { 0, 0, 0, 0 };

        this.setToNoEncoder();
        this.setMaxPower();
        while (opModeIsActive() && !distanceReached(distanceInTicks)) {
            cv.add(this.getVelocity());
            idle();
        }
        this.stopPower();

        telemetry.addData("Count:  ", this.cv.size());
        for(double[] cvItem : cv) {
            for (int i = 0; i < cvItem.length; i++) {
                if (cvItem[i] > maxV[i]) {
                    maxV[i] = cvItem[i];
                }

                telemetry.addData("Maximum Velocity", "%.04f, %.04f, %.0f, %.0f",
                        maxV[0], maxV[1], maxV[2], maxV[3]);
            }
        }

        currentPosition = this.getCurrentPosition();
        telemetry.addData("Current Position", "%d, %d, %d, %d",
                currentPosition[0], currentPosition[1], currentPosition[2], currentPosition[3]);

        //  Regardless of your maximum velocity, you can set the position PIDF values to:  P = 5.0
        //  32767 is the same for ALL motors;
        for (int i = 0; i < 4; i++) {
            F[i] = 32767 / maxV[i];
            P[i] = 0.1 * F[i];
            I[i] = 0.1 * P[i];
            D[i] = 0;
        }

        telemetry.addData("P: ", "%.04f, %.04f, %.0f, %.0f",
                P[0], P[1], P[2], P[3]);

        telemetry.addData("I: ", "%.04f, %.04f, %.0f, %.0f",
                I[0], I[1], I[2], I[3]);

        telemetry.addData("D: ", "%.04f, %.04f, %.0f, %.0f",
                D[0], D[1], D[2], D[3]);

        telemetry.addData("F: ", "%.04f, %.04f, %.0f, %.0f",
                F[0], F[1], F[2], F[3]);

        telemetry.update();

    }

    private void setToEncoderMode() {

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        for(int i = 0; i < 4; i++) {
            leftFrontMotor.setVelocityPIDFCoefficients(P[i], I[i], D[i], F[i]);
            leftFrontMotor.setPositionPIDFCoefficients(5.0);

            rightFrontMotor.setVelocityPIDFCoefficients(P[i], I[i], D[i], F[i]);
            rightFrontMotor.setPositionPIDFCoefficients(5.0);

            leftBackMotor.setVelocityPIDFCoefficients(P[i], I[i], D[i], F[i]);
            leftBackMotor.setPositionPIDFCoefficients(5.0);

            rightBackMotor.setVelocityPIDFCoefficients(P[i], I[i], D[i], F[i]);
            rightBackMotor.setPositionPIDFCoefficients(5.0);
        }
    }

    private void setToNoEncoder() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setupMotors() {

        leftFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get(Constants.LEFT_FRONT_MOTOR_NAME);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackMotor = (DcMotorEx) hardwareMap.dcMotor.get(Constants.LEFT_BACK_MOTOR_NAME);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get(Constants.RIGHT_FRONT_MOTOR_NAME);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        rightBackMotor = (DcMotorEx) hardwareMap.dcMotor.get(Constants.RIGHT_BACK_MOTOR_NAME);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    //  Returns ticks per second;
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

    private boolean distanceReached(int ticks) {
        return (leftFrontMotor.getCurrentPosition() >= ticks || rightFrontMotor.getCurrentPosition() >= ticks || leftBackMotor.getCurrentPosition() >= ticks || rightBackMotor.getCurrentPosition() >= ticks);
    }

    private void setMaxPower() {
        this.leftFrontMotor.setPower(1.0);
        this.rightFrontMotor.setPower(1.0);
        this.leftBackMotor.setPower(1.0);
        this.rightBackMotor.setPower(1.0);
    }

    private void stopPower() {
        this.leftFrontMotor.setPower(0);
        this.rightFrontMotor.setPower(0);
        this.leftBackMotor.setPower(0);
        this.rightBackMotor.setPower(0);
    }

    private void setVelocity(double[] velocity) {
        this.leftFrontMotor.setVelocity(velocity[0]);
        this.rightFrontMotor.setVelocity(velocity[1]);
        this.leftBackMotor.setVelocity(velocity[2]);
        this.rightBackMotor.setVelocity(velocity[3]);
    }

    private int toTicks(double distanceInInches) {
        //  1120 ticks per rotation for Neverest motor;
        //  Gear:  45:35
        //  4 inch diameter;  2" radius;  circumference:  12.5714;

        int ticks = 1120;        //  1120 ticks equals 1 rotation;
        double gearRatio = 45.0/35.0;  //  rotations;
        double wheelCircumference = 4*Math.PI;  //  4" diameters;
        double rotation = distanceInInches / wheelCircumference;
        return (int) ((rotation / gearRatio) * ticks);
    }
}