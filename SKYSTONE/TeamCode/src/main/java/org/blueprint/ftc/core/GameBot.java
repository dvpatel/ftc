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
    private Driver driver;

    //  private TouchSensorController touch;

    private GamepadDriver gamepadDriver;

    private ServoController shortArmServo;

    private SkystoneDetector skystonDetector;

    private IntakeSystem intakeSystem;

    private LiftSystem liftSystem;

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

        //  IMU ;  DON'T SET MODE.
        this.imu = new IMUController(hardwareMap);

        //  Setup short arm servo;
        this.shortArmServo = new ServoController(this.hardwareMap, Constants.SHORT_ARM_SERVO);


        //  Uncomment when TouchSensor is attached;
        //  this.touch = new TouchSensorController(hardwareMap);

        //  Skystone detector, activate when ready ;
        // this.skystonDetector = new SkystoneDetector(hardwareMap);

        //  Drive using gamepad ;
        this.gamepadDriver = new GamepadDriver(this.driver);

        //  IntakeSystem ;
        this.intakeSystem = new IntakeSystem(this.hardwareMap);

        //  LiftSystem
        this.liftSystem = new LiftSystem(hardwareMap);
    }

    private void waitForCalibration() throws InterruptedException {
    }

    public ElapsedTime getTimer() {
        return this.period;
    }

    public ColorSensorController getColorSensorController() {
        this.colorSensor.ledOn();
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
        MotorControllerEx motorPID = new MotorControllerEx();
        motorPID.enableDrivePID(Constants.MOTOR_MAX_VELOCITY);
        return motorPID;
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


    public LiftSystem getLiftSystem() {
        return this.liftSystem;
    }

    public SimpleMotor getLinearSlideMotor() {
        return this.liftSystem.getLinearSlideMotor();
    }

    public ServoController getLinearSlideServo() {
        return this.liftSystem.getLinearSlideServo();
    }

    public ServoController getLinearArmServo() {
        return this.liftSystem.getLinearArmServo();
    }

    public IntakeSystem getIntakeSystem() {
        return this.intakeSystem;
    }

}

