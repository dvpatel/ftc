package org.blueprint.ftc.core;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Config
public final class Constants {

    //  Component names configured in RobotDriver?
    public static final String LEFT_FRONT_MOTOR_NAME = "left_front_motor" ;
    public static final String RIGHT_FRONT_MOTOR_NAME = "right_front_motor" ;
    public static final String LEFT_BACK_MOTOR_NAME = "left_back_motor" ;
    public static final String RIGHT_BACK_MOTOR_NAME = "right_back_motor" ;

    // Left and right motors for intake system
    public static final String INTAKE_LEFT_MOTOR = "intake_left_motor";
    public static final String INTAKE_RIGHT_MOTOR = "intake_right_motor";

    public static final String LINEAR_SLIDE_MOTOR_NAME = "linear_slide_motor";
    public static final String LINEAR_SLIDE_SERVO = "linear_slide_servo";
    public static final String LINEAR_ARM_SERVO = "linear_arm_servo";

    public static final String TOUCH_SENSOR_NAME = "touch_sensor" ;
    public static final String COLOR_SENSOR_NAME = "colorSensor";
    public static final String IMU_NAME = "imu" ;
    public static final String SHORT_ARM_SERVO = "short_arm_servo";

    //  For AndyMark motor;  1 rev = 1120 ticks ;
    public static final int WHEEL_DIAMETER = 4; // inches ;
    public static final int WHEEL_WIDTH = 2; // inches ;

    public static final int MOTOR_TICK_COUNT = 1120;   //  ticks per revoluion;
    public static final int MOTOR_GEAR_RATIO = 40;  //  NeverRest 40; 40:1 gearbox
    public static final int MOTOR_PPR = 280;  //  NeverRest 40; 40:1 gearbox;  40*7
    public static final int MOTOR_NO_LOAD_RPM = 160;

    public static int MOTOR_MAX_VELOCITY = 2900;  //  ticks per second;
    public static int DEFAULT_VELOCITY = (int) (0.90*MOTOR_MAX_VELOCITY);  //  ticks per second;

    public static final double TICK_DIAMETER_RATIO = (Constants.MOTOR_TICK_COUNT / (Math.PI * Constants.WHEEL_DIAMETER));
    public static final double DRIVETRAIN_GEAR_RATIO = 45.0/35.0;  //  Ratio with modified gears;
    public static final double TICK_GEAR_RATIO = Constants.TICK_DIAMETER_RATIO / Constants.DRIVETRAIN_GEAR_RATIO ;

    //  For AndyMark motor connected to linear slide system
    public static final int SIMPLE_WHEEL_DIAMETER = 2; // inches ;
    public static final double SIMPLE_TICK_DIAMETER_RATIO = (Constants.MOTOR_TICK_COUNT / (Math.PI * Constants.SIMPLE_WHEEL_DIAMETER));

    public static final int COLOR_ALPHA=0 ;
    public static final int COLOR_RED=1 ;
    public static final int COLOR_GREEN=2 ;
    public static final int COLOR_BLUE=3 ;

    //  Hue:  340 <  20; sat: 0.6 -->  RED
    //  Hue:  200 - 275; sat: 0.6 -->  Blue
    public static float TARGET_COLOR_BLUE_HUE_LOW = 165;
    public static float TARGET_COLOR_BLUE_HUE_HIGH = 265;

    public static float TARGET_COLOR_RED_HUE_LOW = 320;
    public static float TARGET_COLOR_RED_HUE_HIGH = 15;

    public static float TARGET_COLOR_SATURATION=0.6f ;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    public static double SCALE_FACTOR = 255;

    //  Will need to tune these for driving straight;  See MaxVelocityTest.java
    public static double PID_DRIVE_KP = 1.18 ;
    public static double PID_DRIVE_KI = 0.118 ;
    public static double PID_DRIVE_KD = 0.0;
    public static double PID_DRIVE_KF = 11.7867;

    //  Will need to tune these for turning;
    // Speed of wheels while slowing down
    public static double PID_ROTATE_KP = 0.030;       //  0.009

    // Do not need - once the robot reaches 90°, the angle does not change.
    public static double PID_ROTATE_KI = 0;           //  0.0000374

    // How much the robot slows down before it reaches 90°
    public static double PID_ROTATE_KD = 0.025;

    public static final String VUFORIA_KEY =
            "ARBNGpP/////AAABmbLvzx0Qekiui2o+DSSa3YJIIuD7Q0UL6sLKYRh6/OCm/uvQLlRLPNs/o72itb3SXgG71435htgeXTLgMciuPUca8vG5BbLoR5k9K5L6pbe8XLD9VFAG4Llh55ETmOQzz+S7yyjN69HtY34ahSjsi4bzZzwrfeTrTsCPfa1ZTAdf6MxWbZ5yn6LKmanzxLbnmBiftmRbgVVtxeMbOdxPv/f2uxXWqnKEHz5/LDvoacDFVQwu07AnvUXk0cDRSKEObQs5lE+IjdxSbYMOHPYbJy9jWf+2tZURyVZF1atz0nHaW1yra8YXg0HYQvWDzkt9+2S831dsB25sJElDK4xLGFFb/GVCSFGfnjvRHgbvD1AP";
    public static boolean PHONE_IS_PORTRAIT = false;
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final float MM_PER_INCHES = 25.4f;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    public static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    public static final double HEADING_THRESHOLD = 1 ;      // As tight as we can make it with an integer gyro
    public static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    public static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    //  tile is 22.75" by 22.75';
    public static final float TILE_SIZE = 22.75f;

    //  DriveTrain size;
    public static final float DRIVETRAIN_LENGTH = 17.75f;
    public static final float DRIVETRAIN_WIDTH = 17.10f;
    public static final float DRIVETRAIN_HEIGHT = 3.50f;

    //  Tune this for gamepad.
    public static double DEADZONE = 0.15;

}
