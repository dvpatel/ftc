package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class GameBot {

    /* local OpMode members. */
    private HardwareMap hardwareMap;

    private ColorSensorController colorSensor;
    private IMUController imu;
    private MotorControllerEx motorPID;
    private Driver driver;
    private TouchSensorController touch;
    private GamepadDriver gamepadDriver;
    private ServoController shortArmServo;

    private SkystoneDetector skystonDetector;

    private static final double MID_SERVO = 0.5;
    private static final double ARM_UP_POWER = 0.45;
    private static final double ARM_DOWN_POWER = -0.45;

    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public GameBot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) throws InterruptedException {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        // get a reference to the color sensor.
        this.colorSensor = new ColorSensorController(hardwareMap);

        //  Build driver ;
        this.driver = new Driver(hardwareMap);

        //  DP:  Causing init to fail;
        //  IMU ;  Calibrate Gyro ;
        this.imu = new IMUController(hardwareMap);

        //   PID Logic for DC Motors;  Default 0.30 power ;
        this.motorPID = new MotorControllerEx();
        this.motorPID.enableDrivePID(Constants.DEFAULT_POWER);

        //  Setup short arm servo;
        this.shortArmServo = new ServoController(this.hardwareMap);

        //  Uncomment when TouchSensor is attached;
        //  this.touch = new TouchSensorController(hardwareMap);

        //  Skystone detector, activate when ready ;
        //  this.skystonDetector = new SkystoneDetector(hardwareMap);

        //  Drive using gamepad ;
        this.gamepadDriver = new GamepadDriver(this.driver);


    }

    private void waitForCalibration() throws InterruptedException {
    }

    public ElapsedTime getTimer() {
        return this.period;
    }

    public ColorSensorController getColorSensorController() {
        return this.colorSensor;
    }

    public IMUController getIMUController() {
        return this.imu;
    }

    public TouchSensorController getTouchSensorController() {
        return this.touch;
    }

    public MotorControllerEx getMotorPID() {
        return this.motorPID;
    }

    public Driver getDriver() {
        return this.driver;
    }

    public SkystoneDetector getSkystoneDetector() {
        return this.skystonDetector;
    }

    public GamepadDriver getGamepadDriver() {
        return this.gamepadDriver;
    }

    public ServoController getShortArmServo() {
        return this.shortArmServo;
    }


}

