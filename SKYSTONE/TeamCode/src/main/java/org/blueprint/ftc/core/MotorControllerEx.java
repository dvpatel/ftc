package org.blueprint.ftc.core;

import com.qualcomm.robotcore.util.Range;

public class MotorControllerEx {

    private PIDController pidDrive;

    private PIDController pidRotate;

    private boolean enableRotatePidCalled;

    //  private double power = .30;
    //  private double correction ;

    public MotorControllerEx() {

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        this.pidDrive = new PIDController(Constants.PID_DRIVE_KP, Constants.PID_DRIVE_KI, Constants.PID_DRIVE_KD);


        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        this.pidRotate = new PIDController(Constants.PID_ROTATE_KP, Constants.PID_ROTATE_KI, Constants.PID_ROTATE_KD);
    }

    public void enableDrivePID(double velocity) {
        // Set up parameters for driving in a straight line.
        this.pidDrive.setSetpoint(0);            // Want to set PID value to 0;

        // always positive;  power output
        this.pidDrive.setOutputRange(0, Range.clip(velocity, -Constants.MOTOR_MAX_VELOCITY, Constants.MOTOR_MAX_VELOCITY));

        this.pidDrive.setInputRange(-90, 90);    // always positive;  angle
        this.pidDrive.enable();                  //  Enable PID calculation
    }


    public void enableRotatePID(double degrees, double velocity) {

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        this.pidRotate.reset();
        this.pidRotate.setSetpoint(degrees);
        this.pidRotate.setInputRange(0, degrees);
        this.pidRotate.setOutputRange(0, Range.clip(velocity, -Constants.MOTOR_MAX_VELOCITY, Constants.MOTOR_MAX_VELOCITY));
        this.pidRotate.setTolerance(0.5);
        this.pidRotate.enable();

        this.enableRotatePidCalled = true;
    }

    //  Customize pid for drive ;
    public void setDrivePID(double rkp, double rki, double rkd) {
        this.pidDrive = new PIDController(rkp, rki, rkd);
    }

    //  Customize pid for rotation ;
    public void setRotatePID(double rkp, double rki, double rkd) {
        this.pidRotate = new PIDController(rkp, rki, rkd);
    }


    public void disablePID(){
        this.pidDrive.disable();
    }

    //  logic for driving straight; corrects if angle is not zero ;
    public double[] calculateDriveCorrection(double velocity, double angle) {
        // Use PID with imu input to drive in a straight line.

        double correction = this.pidDrive.performPID(angle);
        return new double[]{(velocity - correction), (velocity + correction), correction};
    }


    public double calculateRotateCorrection(double degrees, double angle, double velocity) {

        //  Must call this before opModeActive
        if (!this.enableRotatePidCalled) {
            this.enableRotatePID(degrees, velocity);
            this.enableRotatePidCalled = true;
        }

        return this.pidRotate.performPID(angle);
    }

    public boolean angleOnTarget() {
        return this.pidRotate.onTarget() ;
    }

}