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

        this.colorSensor = new ColorSensorController(hardwareMap);
        this.colorSensor.ledOn();

        this.skystonDetector = new SkystoneDetector(hardwareMap);

        this.foundationSystem = new FoundationSystem(this.hardwareMap);

        this.liftSystem = new LiftSystem(hardwareMap);

        this.intakeSystem = new IntakeSystem(this.hardwareMap);
    }

    public MotorControllerEx getMotorPID() {

        //  MotorPID cannot be a singleton;  Otherwise internal state is maintained and prevents multiple turns;
        MotorControllerEx motorPID = new MotorControllerEx();
        return motorPID;
    }

    public ColorSensorController getColorSensorController() {
        return this.colorSensor;
    }

    public IMUController getIMUController() {
        return this.imu;
    }

    public Driver getDriver() {
        return this.driver;
    }

    public SkystoneDetector getSkystoneDetector() {
        //  Skystone detector, activate when ready ;
        return this.skystonDetector;
    }

    public GamepadDriver getGamepadDriver() {
        return this.gamepadDriver;
    }

    public FoundationSystem getFoundationSystem() {
        return this.foundationSystem;
    }

    public LiftSystem getLiftSystem() {
        return this.liftSystem;
    }

    public IntakeSystem getIntakeSystem() {
        return this.intakeSystem;
    }
}

