package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Rosie
 */
public class GameBot {

    /* local OpMode members. */
    private HardwareMap hardwareMap;

    private ColorSensorController colorSensor;
    private IMUController imu;
    //  private MotorControllerEx motorPID;
    private Driver driver;

    //  Motor to power linear slide system
    private SimpleMotor linearSlideMotor;

    //  private TouchSensorController touch;

    private GamepadDriver gamepadDriver;

    private ServoController shortArmServo;
    private ServoController linearSlideServo;
    private ServoController linearArmServo;

    private SkystoneDetector skystonDetector;

    private IntakeSystem intakeSystem;

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

        //  motor to control linear slide system
        this.linearSlideMotor = new SimpleMotor(hardwareMap, Constants.LINEAR_SLIDE_MOTOR_NAME);

        //  IMU ;  DON'T SET MODE.
        this.imu = new IMUController(hardwareMap);

        //   PID Logic for DC Motors;  Default 0.30 power ;
        //  this.motorPID = new MotorControllerEx();
        //  this.motorPID.enableDrivePID(Constants.DEFAULT_POWER);

        //  Setup short arm servo;
        this.shortArmServo = new ServoController(this.hardwareMap, Constants.SHORT_ARM_SERVO);

        //  Servo attached to linear slide system;
        this.linearSlideServo = new ServoController(this.hardwareMap, Constants.LINEAR_SLIDE_SERVO);

        //  Servo attached to linear arm system;
        this.linearArmServo = new ServoController(this.hardwareMap, Constants.LINEAR_ARM_SERVO);

        //  Uncomment when TouchSensor is attached;
        //  this.touch = new TouchSensorController(hardwareMap);

        //  Skystone detector, activate when ready ;
        // this.skystonDetector = new SkystoneDetector(hardwareMap);

        //  Drive using gamepad ;
        this.gamepadDriver = new GamepadDriver(this.driver);

        //  IntakeSystem ;
        this.intakeSystem = new IntakeSystem(this.hardwareMap);

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

    //  public TouchSensorController getTouchSensorController() {
    //     return this.touch;
    //  }

    public MotorControllerEx getMotorPID() {

        //  MotorPID cannot be a singleton;  Otherwise internal state is maintained and prevents multiple turns;

        //  return this.motorPID;
        MotorControllerEx motorPID = new MotorControllerEx();
        motorPID.enableDrivePID(Constants.DEFAULT_POWER);
        return motorPID;
    }

    public Driver getDriver() {
        return this.driver;
    }

    public SimpleMotor getLinearSlideMotor() {
        return this.linearSlideMotor;
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

    public ServoController getLinearSlideServo() {
        return this.linearSlideServo;
    }

    public ServoController getLinearArmServo() {
        return this.linearArmServo;
    }

    public IntakeSystem getIntakeSystem() {
        return this.intakeSystem;
    }

}

