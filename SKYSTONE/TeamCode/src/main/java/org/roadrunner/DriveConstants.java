package org.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {

    /*
     * The type of motor used on the drivetrain. While the SDK has definitions for many common
     * motors, there may be slight gear ratio inaccuracies for planetary gearboxes and other
     * discrepancies. Additional motor types can be defined via an interface with the
     * @DeviceProperties and @MotorType annotations.
     */
    public static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest20Orbital.class);

    /*
     * Set the first flag appropriately. If using the built-in motor velocity PID, update
     * MOTOR_VELO_PID with the tuned coefficients from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = true;
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(10.0, 0.25, 0);

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);

    //  PID for turning;
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(5.0, 0, 0.2);

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 2;             //  4 inches diameter;
    public static double GEAR_RATIO = 45.0 / 35.0;     //  output (wheel) speed / input (motor) speed;  40:1 for Andymark
    public static double TRACK_WIDTH = 17.25;          //  Based TrackWidthTuner, 22.31;  17.25 from TurnTest
    public static double LATERAL_MULTIPLIER = 1.9;     //  Tuned for 24"
    public static double WHEEL_BASE = 13.582;
    public static double STRAFE_FACTOR = 46.0/24.0;    //  FOR STRAFING; HACK


    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 1.0 / rpmToVelocity(getMaxRpm());
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling).
     *
     * 537.6 ticks / 1.2 rotation;   36.67 ticks / inches;  For Orbital 20
     * maxRPM:  537.6*340/60 = 3046.4 rev/sec;  71.23 inches/seconds
     *     *
     * 2820 ticks / seconds -->  65.94 max vel, @85% 56.05
     *
     */
    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            56.05, 56.05, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTOR_CONFIG.getTicksPerRev();
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return MOTOR_CONFIG.getMaxRPM() *
                (RUN_USING_ENCODER ? MOTOR_CONFIG.getAchieveableMaxRPMFraction() : 1.0);
    }

    public static double getTicksPerSec() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        return (MOTOR_CONFIG.getMaxRPM() * MOTOR_CONFIG.getTicksPerRev() / 60.0);
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / getTicksPerSec();
    }
}