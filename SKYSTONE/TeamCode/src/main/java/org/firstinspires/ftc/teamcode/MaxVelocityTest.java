package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.blueprint.ftc.core.Constants;

import java.util.ArrayList;
import java.util.List;

//  NOTE:  TEST WITH ROBOT LOAD!!!  DON'T PUT ON SKYSTONE BLOCK

@Config
@TeleOp(name = "MaxVelocityTest")
@Disabled
public class MaxVelocityTest extends LinearOpMode {

    public static double DISTANCE = 24; //  inches
    public static double POWER = 1.0;
    public static int COUNT = 1;


    //  Used for Run-To-Pos;
    public static double posP = 1.7;

    //  Used for Run-Using-Encoder;
    public static double p = 1.1538;
    public static double i = p / 10.0;
    public static double d = 0;
    public static double f = 10.0 * p;

    //  leftFront, rightFront, leftBack, rightBack;
    private static double[] P = {p, p, p, p};
    private static double[] I = {i, i, i, i};
    private static double[] D = {d, d, d, d};
    private static double[] F = {f, f, f, f};

    //  public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(1.0, 0.152, 0.0375);
    public double[] maxVelocity = {2820, 2820, 2820, 2820};
    int[] currentPosition = {0, 0, 0, 0};
    double[] currentVelocity = {0, 0, 0, 0};

    private List<DcMotorEx> motors = new ArrayList<DcMotorEx>();
    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.setupMotors();

        waitForStart();

        if (opModeIsActive()) {
            this.drivingTest();
        }

        this.stop();
    }

    private void drivingTest() {

        telemetry.addData("Test Type:", "Driving");
        // telemetry.addData("Default Velocity Coeffs:  ", this.getVelocityPIDFCoefficients());
        // telemetry.addData("Default Position Coeffs:  ", this.getPositionPIDFCoefficients());
        // telemetry.update();

        //  Run this method to determine PIDF values;  Make sure robot is running with load!
        //  Will set maximum velocity and PIDF for each wheel!
        //  this.calculatePIDF(COUNT);

        //  Update Constants file:  PID, MAX_Velocity;

        //  Run this to validate PIDF values;  Besure to update maxVelocity and PIDF variables
        this.testWithPIDFValues();

    }

    private void testWithPIDFValues() {

        int distanceInTicks = this.toTicks(DISTANCE);

        double[] velocity = {0, 0, 0, 0};
        for (int i = 0; i < 4; i++) {
            velocity[i] = 1 * maxVelocity[i];  //  Don't change.
        }

        //  Verify PID values;
        this.setToEncoderMode();
        this.setTicksToTargets(distanceInTicks);
        this.setRunToPositionMode();

        //  Set at 50% max velocity;
        this.setVelocity(velocity);

        //  NOTE:  TEST THIS with both distanceReached methods;
        while (opModeIsActive() && this.motorsBusy()) {

            currentVelocity = this.getVelocity();
            currentPosition = this.getCurrentPosition();

            telemetry.addData("Targeted Velocity", "%.04f, %.04f, %.0f, %.0f",
                    velocity[0], velocity[1], velocity[2], velocity[3]);

            telemetry.addData("Current Velocity", "%.04f, %.04f, %.0f, %.0f",
                    currentVelocity[0], currentVelocity[1], currentVelocity[2], currentVelocity[3]);

            telemetry.addData("Delta Velocity", "%.04f, %.04f, %.0f, %.0f",
                    (velocity[0] - currentVelocity[0]),
                    (velocity[1] - currentVelocity[1]),
                    (velocity[2] - currentVelocity[2]),
                    (velocity[3] - currentVelocity[3]));

            //  Returns ticks per second;
            telemetry.addData("Targeted Ticks:", distanceInTicks);
            telemetry.addData("Targeted Inches:", toInches(distanceInTicks));

            telemetry.addData("CurrentPosition", "%d, %d, %d, %d",
                    currentPosition[0], currentPosition[1], currentPosition[2], currentPosition[3]);

            telemetry.addData("CurrentPosition Inches", "%.04f, %.04f, %.0f, %.0f",
                    toInches(currentPosition[0]),
                    toInches(currentPosition[1]),
                    toInches(currentPosition[2]),
                    toInches(currentPosition[3]));

            telemetry.update();
        }

        sleep(5000);

        currentPosition = this.getCurrentPosition();

        telemetry.addData("Targeted Ticks:", distanceInTicks);
        telemetry.addData("Targeted Inches:", toInches(distanceInTicks));

        telemetry.addData("CurrentPosition", "%d, %d, %d, %d",
                currentPosition[0], currentPosition[1], currentPosition[2], currentPosition[3]);

        telemetry.addData("CurrentPosition Inches", "%.04f, %.04f, %.0f, %.0f",
                toInches(currentPosition[0]),
                toInches(currentPosition[1]),
                toInches(currentPosition[2]),
                toInches(currentPosition[3]));

        telemetry.update();


        this.stopPower();
    }

    private void calculatePIDF(int count) {

        int distanceInTicks = this.toTicks(DISTANCE);

        List<double[]> cv = new ArrayList<double[]>();

        //  Capture data in both directions;
        boolean isReverse = false;
        for (int i = 0; i < count; i++) {
            this.setToNoEncoder();
            this.setMaxPower(isReverse);
            while (opModeIsActive() && !distanceReached(distanceInTicks)) {
                cv.add(this.getVelocity());
                idle();
            }
            this.stopPower();
            isReverse = !isReverse;

            sleep(2000);
        }

        for (double[] cvItem : cv) {
            for (int i = 0; i < cvItem.length; i++) {
                if (cvItem[i] > maxVelocity[i]) {
                    maxVelocity[i] = cvItem[i];
                }
            }
        }

        telemetry.addData("Max Velocity", "%.04f, %.04f, %.0f, %.0f",
                maxVelocity[0], maxVelocity[1], maxVelocity[2], maxVelocity[3]);

        telemetry.addData("Distance Ticks:", distanceInTicks);
        telemetry.addData("Distance Inches:", toInches(distanceInTicks));

        currentPosition = this.getCurrentPosition();
        telemetry.addData("Current Position", "%d, %d, %d, %d",
                currentPosition[0], currentPosition[1], currentPosition[2], currentPosition[3]);

        telemetry.addData("Current Position Inches", "%.04f, %.04f, %.0f, %.0f",
                toInches(currentPosition[0]),
                toInches(currentPosition[1]),
                toInches(currentPosition[2]),
                toInches(currentPosition[3]));


        //  Capture data for all four wheels;
        //  Regardless of your maximum velocity, you can set the position PIDF values to:  P = 5.0
        //  32767 is the same for ALL motors;
        for (int i = 0; i < 4; i++) {
            F[i] = 32767 / maxVelocity[i];
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

    private void setupMotors() {

        leftFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get(Constants.LEFT_FRONT_MOTOR_NAME);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motors.add(leftFrontMotor);

        leftBackMotor = (DcMotorEx) hardwareMap.dcMotor.get(Constants.LEFT_BACK_MOTOR_NAME);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motors.add(leftBackMotor);

        rightFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get(Constants.RIGHT_FRONT_MOTOR_NAME);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motors.add(rightFrontMotor);

        rightBackMotor = (DcMotorEx) hardwareMap.dcMotor.get(Constants.RIGHT_BACK_MOTOR_NAME);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motors.add(rightBackMotor);
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

    private void setToEncoderMode() {

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontMotor.setVelocityPIDFCoefficients(P[0], I[0], D[0], F[0]);
        leftFrontMotor.setPositionPIDFCoefficients(posP);

        rightFrontMotor.setVelocityPIDFCoefficients(P[1], I[1], D[1], F[1]);
        rightFrontMotor.setPositionPIDFCoefficients(posP);

        leftBackMotor.setVelocityPIDFCoefficients(P[2], I[2], D[2], F[2]);
        leftBackMotor.setPositionPIDFCoefficients(posP);

        rightBackMotor.setVelocityPIDFCoefficients(P[3], I[3], D[3], F[3]);
        rightBackMotor.setPositionPIDFCoefficients(posP);
    }

    public void setTicksToTargets(int ticks) {
        this.leftFrontMotor.setTargetPosition(ticks);
        this.rightFrontMotor.setTargetPosition(ticks);
        this.leftBackMotor.setTargetPosition(ticks);
        this.rightBackMotor.setTargetPosition(ticks);
    }

    public void setRunToPositionMode() {
        this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //  Returns ticks per second;
    private double[] getVelocity() {
        double[] v = {
                Math.abs(leftFrontMotor.getVelocity()),
                Math.abs(rightFrontMotor.getVelocity()),
                Math.abs(leftBackMotor.getVelocity()),
                Math.abs(rightBackMotor.getVelocity())};
        return v;
    }

    private void setVelocity(double[] velocity) {
        this.leftFrontMotor.setVelocity(velocity[0]);
        this.rightFrontMotor.setVelocity(velocity[1]);
        this.leftBackMotor.setVelocity(velocity[2]);
        this.rightBackMotor.setVelocity(velocity[3]);
    }

    private int[] getCurrentPosition() {
        int[] v = {
                leftFrontMotor.getCurrentPosition(),
                rightFrontMotor.getCurrentPosition(),
                leftBackMotor.getCurrentPosition(),
                rightBackMotor.getCurrentPosition()};
        return v;
    }

    public String[] getVelocityPIDFCoefficients() {
        String[] velocityCoeffs = {
                this.leftFrontMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString(),
                this.rightFrontMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString(),
                this.leftBackMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString(),
                this.rightBackMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString(),
        };
        return velocityCoeffs;
    }

    public String[] getPositionPIDFCoefficients() {
        String[] coeffs = {
                this.leftFrontMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString(),
                this.rightFrontMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString(),
                this.leftBackMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString(),
                this.rightBackMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString(),
        };
        return coeffs;
    }

    private boolean distanceReached(int ticks) {

        boolean result = false;
        for (DcMotorEx motor : motors) {
            if (Math.abs(motor.getCurrentPosition()) >= ticks) {
                this.stopPower(motor);
                result = true;
            } else {
                result = false;
            }
        }

        return result;

    }

    private boolean distanceReachedUnified(int ticks) {
        return (leftFrontMotor.getCurrentPosition() >= ticks || rightFrontMotor.getCurrentPosition() >= ticks || leftBackMotor.getCurrentPosition() >= ticks || rightBackMotor.getCurrentPosition() >= ticks);
    }

    public boolean motorsBusy() {
        return this.leftFrontMotor.isBusy() &&
                this.rightFrontMotor.isBusy() &&
                this.leftBackMotor.isBusy() &&
                this.rightBackMotor.isBusy();
    }

    private void setMaxPower(boolean isReverse) {

        double[] power = {POWER, POWER, POWER, POWER};
        if (isReverse) {
            for (int i = 0; i < power.length; i++) {
                power[i] = -POWER;
            }
        }

        this.leftFrontMotor.setPower(power[0]);
        this.rightFrontMotor.setPower(power[1]);
        this.leftBackMotor.setPower(power[2]);
        this.rightBackMotor.setPower(power[3]);

    }

    private void stopPower() {
        this.leftFrontMotor.setPower(0);
        this.rightFrontMotor.setPower(0);
        this.leftBackMotor.setPower(0);
        this.rightBackMotor.setPower(0);
    }

    private void stopPower(DcMotorEx motor) {
        motor.setPower(0);
    }

    private double toInches(int ticks) {
        return (ticks / Constants.TICK_GEAR_RATIO);
    }

    private int toTicks(double distanceInInches) {
        //  1120 ticks per rotation for Neverest motor;
        //  Gear:  45:35
        //  4 inch diameter;  2" radius;  circumference:  12.5714;

        double ticks = Constants.MOTOR_TICK_COUNT;        //  1120 ticks equals 1 rotation;
        double gearRatio = Constants.DRIVETRAIN_GEAR_RATIO;  //  rotations;
        double wheelCircumference = 4 * Math.PI;  //  4" diameters;
        double rotation = distanceInInches / wheelCircumference;
        return (int) ((rotation / gearRatio) * ticks);
    }
}