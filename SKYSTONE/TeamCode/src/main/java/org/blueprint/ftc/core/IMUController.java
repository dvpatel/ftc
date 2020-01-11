package org.blueprint.ftc.core;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.blueprint.ftc.core.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class IMUController {

    private BNO055IMU imu ;
    private Orientation lastAngles ;

    private double globalAngle ;

    public IMUController(HardwareMap hardwareMap) {
        this(hardwareMap, Constants.IMU_NAME) ;
    }

    public IMUController(HardwareMap hardwareMap, String deviceName) {
        this.imu = hardwareMap.get(BNO055IMU.class, deviceName);
        this.initParameters() ;

        this.lastAngles = new Orientation();
    }

    private void initParameters() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        this.imu.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.
        //  NOTE:  Not needed if mode is not set.  Automatically calibrates sensor;
        //  Successful, calibrated reading of getCalibrationStatus:  s0 g3 a0 m0;
    }

    private BNO055IMU getIMU() {
        return this.imu ;
    }

    public boolean isCalibrated() {
        return this.imu.isGyroCalibrated() ;
    }

    public String getCalibrationStatus() {
        return this.imu.getCalibrationStatus().toString() ;
    }

    public void resetAngle()
    {
        this.lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.globalAngle = 0;
    }

    //  Yaw angle;
    public double getYaw() {
        return this.getAngle();
    }

    /**
     * Get current cumulative angle rotation from last reset.  Yaw angle;
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - this.lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        this.globalAngle += deltaAngle;

        this.lastAngles = angles;

        return this.globalAngle;
    }

    private double getGlobalAngle() {
        return this.globalAngle ;
    }

    public float getFirstAngle() {
        return this.lastAngles.firstAngle ;
    }


}
