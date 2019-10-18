package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorControllerEx {

    private PIDController pidDrive;

    private PIDController pidRotate;

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

    public void enableDrivePID(double power) {
        // Set up parameters for driving in a straight line.
        this.pidDrive.setSetpoint(0);            // Want to set PID value to 0;
        this.pidDrive.setOutputRange(0, power);  // always positive;  power output
        this.pidDrive.setInputRange(-90, 90);    // always positive;  angle
        this.pidDrive.enable();                  //  Enable PID calculation
    }


    public void enableRotatePID(double degrees, double power) {
        this.pidRotate.reset();
        this.pidRotate.setSetpoint(degrees);
        this.pidRotate.setInputRange(0, degrees);
        this.pidRotate.setOutputRange(0, power);
        this.pidRotate.setTolerance(1);
        this.pidRotate.enable();
    }

    public void disablePID(){
        this.pidDrive.disable();
    }

    //  logic for driving straight; corrects if angle is not zero ;
    public double[] calculateDriveCorrection(double power, double angle) {
        // Use PID with imu input to drive in a straight line.

        double powerCorrection = this.pidDrive.performPID(angle);
        double[] corrections = {(power - powerCorrection), (power + powerCorrection), powerCorrection} ;

        return corrections ;
    }


    public double calculateRotateCorrection(double degrees, double angle, double power) {
        this.enableRotatePID(degrees, power) ;
        return this.pidRotate.performPID(angle);
    }

    public boolean angleOnTarget() {
        return this.pidRotate.onTarget() ;
    }

}