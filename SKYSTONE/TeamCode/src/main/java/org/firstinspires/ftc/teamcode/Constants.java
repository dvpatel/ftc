package org.firstinspires.ftc.teamcode;

public final class Constants {
    public static final String LEFT_FRONT_MOTOR_NAME = "left_front_motor" ;
    public static final String RIGHT_FRONT_MOTOR_NAME = "right_front_motor" ;
    public static final String LEFT_BACK_MOTOR_NAME = "left_back_motor" ;
    public static final String RIGHT_BACK_MOTOR_NAME = "right_back_motor" ;
    public static final String TOUCH_SENSOR_NAME = "touch_sensor" ;
    public static final String IMU_NAME = "imu" ;


    public static final String COLOR_SENSOR_NAME = "colorSensor" ;
    public static final int COLOR_ALPHA=0 ;
    public static final int COLOR_RED=1 ;
    public static final int COLOR_GREEN=2 ;
    public static final int COLOR_BLUE=3 ;


    //  Hue:  340 <  20; sat: 0.6 -->  RED
    //  Hue:  200 - 275; sat: 0.6 -->  Blue
    public static final float TARGET_COLOR_BLUE_HUE_LOW = 200;
    public static final float TARGET_COLOR_BLUE_HUE_HIGH = 275;

    public static final float TARGET_COLOR_RED_HUE_LOW = 340;
    public static final float TARGET_COLOR_RED_HUE_HIGH = 20;

    public static final float TARGET_COLOR_SATURATION=0.6f ;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    public static final double SCALE_FACTOR = 255;


    //  Will need to tune these
    public static double PID_DRIVE_KP = 0.05 ;
    public static double PID_DRIVE_KI = 0 ;
    public static double PID_DRIVE_KD = 0 ;

    //  Will need to tune these;
    public static double PID_ROTATE_KP = 0.003 ;
    public static double PID_ROTATE_KI = 0.00003 ;
    public static double PID_ROTATE_KD = 0 ;


    public static final double COUNTS_PER_MOTOR_REV = 6600 ;    // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 2.0 ;     // This is < 1.0 if geared UP, ???
    public static final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    public static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    public static final double HEADING_THRESHOLD = 1 ;      // As tight as we can make it with an integer gyro
    public static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    public static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

}
