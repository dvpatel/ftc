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

    private GamepadDriver gamepadDriver;

    private SkystoneDetector skystonDetector;

    private IntakeSystem intakeSystem;
    private LiftSystem liftSystem;
    private FoundationSystem foundationSystem;


    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public GameBot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) throws InterruptedException {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        //  IMU ;  DON'T SET MODE.
        this.imu = new IMUController(hardwareMap);


        //  Build driver ;
        this.driver = new Driver(hardwareMap);


        //  Drive using gamepad ;
        this.gamepadDriver = new GamepadDriver(this.driver, this.imu);

    }

    private void waitForCalibration() throws InterruptedException {
    }

    public ElapsedTime getTimer() {
        return this.period;
    }

    public ColorSensorController getColorSensorController() {

        // get a reference to the color sensor.
        if (this.colorSensor == null) {
            this.colorSensor = new ColorSensorController(hardwareMap);
            this.colorSensor.ledOn();
        }

        return this.colorSensor;
    }

    public IMUController getIMUController() {
        return this.imu;
    }

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

        //  Skystone detector, activate when ready ;
        if (this.skystonDetector == null) {
            this.skystonDetector = new SkystoneDetector(hardwareMap);
        }

        return this.skystonDetector;
    }

    public GamepadDriver getGamepadDriver() {
        return this.gamepadDriver;
    }

    public FoundationSystem getFoundationSystem() {
        //  Setup short arm servo;
        if (this.foundationSystem == null) {
            this.foundationSystem = new FoundationSystem(this.hardwareMap);
        }

        return this.foundationSystem;
    }

    public LiftSystem getLiftSystem() {
        //  LiftSystem
        if (this.liftSystem == null) {
            this.liftSystem = new LiftSystem(hardwareMap);
        }

        return this.liftSystem;
    }

    public SimpleMotor getLinearSlideMotor() {
        return this.getLiftSystem().getLinearSlideMotor();
    }

    public ServoController getLinearSlideServo() {
        return this.getLiftSystem().getLinearSlideServo();
    }

    public ServoController getLinearArmServo() {
        return this.getLiftSystem().getLinearArmServo();
    }

    public IntakeSystem getIntakeSystem() {
        //  IntakeSystem ;
        if (this.intakeSystem == null) {
            this.intakeSystem = new IntakeSystem(this.hardwareMap);
        }

        return this.intakeSystem;
    }
}

