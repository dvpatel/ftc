package org.blueprint.ftc.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public class SkystoneDetector {

    private HardwareMap hardwareMap;
    private LinearOpMode myOpMode;

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables targetsSkyStone;
    private List<VuforiaTrackable> allTrackables;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    private boolean targetFound;    // set to true if Vuforia is currently tracking a target
    private String targetName;     // Name of the currently tracked target

    private double robotX;
    private double robotY;
    private double robotZ;
    private double robotAngleX, robotAngleY, robotAngleZ;

    private double robotBearing;   // Robot's rotation around the Z axis (CCW is positive)
    private double targetRange;    // Range from robot's center to target in mm
    private double targetBearing;  // Heading of the target , relative to the robot's unrotated center
    private double relativeBearing;// Heading to the target from the robot's current bearing.


    public SkystoneDetector(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;
        this.webcamName = this.hardwareMap.get(WebcamName.class, Constants.CAMERA_NAME);
    }

    public void initVuforia(LinearOpMode opMode) {

        this.myOpMode = opMode;
        VuforiaLocalizer.Parameters parameters = this.getParameters();

        //  Instantiate the Vuforia engine
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        this.targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        this.allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.

        float[] cameraDisplacement = this.getCameraDisplacement();
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(cameraDisplacement[1], cameraDisplacement[2], cameraDisplacement[0])
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZX,
                        AngleUnit.DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));


        //  final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center, dY
        //  final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground, dZ
        //  final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line, dX
        //  OpenGLMatrix robotFromCamera = OpenGLMatrix
        //          .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
        //          .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));


        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    }

    private VuforiaLocalizer.Parameters getParameters() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */

        VuforiaLocalizer.Parameters parameters;
        if (Constants.CAMERA_MONITOR_ON) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        return parameters;
    }

    //  Get position of camera from robot origin;
    private float[] getCameraDisplacement() {
        //  assume robot is facing front on red alliance
        //  x distance from origin, y distance from front, height
        float[] position = {Constants.CAMERA_DX_MM, Constants.CAMERA_DY_MM, Constants.CAMERA_DZ_MM};
        return position;
    }

    /***
     * See if any of the vision targets are in sight.
     *
     * @return true if any target is found
     */
    public boolean targetsAreVisible() {

        int targetTestID = 0;

        // Check each target in turn, but stop looking when the first target is found.
        while ((targetTestID < 12) && !targetIsVisible(targetTestID)) {
            targetTestID++;
        }

        return (targetFound);
    }

    /***
     * Determine if specified target ID is visible and
     * If it is, retreive the relevant data, and then calculate the Robot and Target locations
     *
     * @param   targetId
     * @return true if the specified target is found
     */
    private boolean targetIsVisible(int targetId) {

        VuforiaTrackable target = targetsSkyStone.get(targetId);
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) target.getListener();
        OpenGLMatrix location = null;

        // if we have a target, look for an updated robot position
        if ((target != null) && (listener != null) && listener.isVisible()) {
            targetFound = true;
            targetName = target.getName();

            // If we have an updated robot location, update all the relevant tracking information
            location = listener.getUpdatedRobotLocation();
            if (location != null) {

                // Create a translation and rotation vector for the robot.
                VectorF trans = location.getTranslation();
                Orientation rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                robotAngleX = rot.firstAngle;
                robotAngleY = rot.secondAngle;
                robotAngleZ = rot.thirdAngle;

                // Robot position is defined by the standard Matrix translation (x and y)
                robotX = trans.get(0);
                robotY = trans.get(1);
                robotZ = trans.get(2);

                // Robot bearing (in +vc CCW cartesian system) is defined by the standard Matrix z rotation
                robotBearing = rot.thirdAngle;

                // target range is based on distance from robot position to origin.
                targetRange = Math.hypot(robotX, robotY);

                // target bearing is based on angle formed between the X axis to the target range line
                targetBearing = Math.toDegrees(-Math.asin(robotY / targetRange));

                // Target relative bearing is the target Heading relative to the direction the robot is pointing.
                relativeBearing = targetBearing - robotBearing;
            }
            targetFound = true;
        } else {
            // Indicate that there is no target visible
            targetFound = false;
            targetName = "None";
        }

        return targetFound;
    }

    public double[] getTargetCoordinatesInInches() {
        //  X, Y, Z, Yaw, Hypotenuse, Theta, relativeBearing
        double[] r = { robotX/mmPerInch, robotY/mmPerInch, robotZ/mmPerInch, robotBearing, targetRange/mmPerInch, targetBearing, relativeBearing};
        return r;
    }

    /***
     * Start tracking Vuforia images
     */
    public void activateTracking() {

        // Start tracking any of the defined targets
        if (targetsSkyStone != null)
            targetsSkyStone.activate();
    }

    public void deactivateTracking() {
        // Start tracking any of the defined targets
        if (targetsSkyStone != null)
            targetsSkyStone.deactivate();
    }

    public String getTargetName() {
        return this.targetName;
    }

    /***
     * Send telemetry data to indicate navigation status
     */
    public void addNavTelemetry() {
        if (targetFound) {
            // Display the current visible target name, robot info, target info, and required robot action.
            myOpMode.telemetry.addData("Visible", targetName);

            this.myOpMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    robotX / mmPerInch, robotY / mmPerInch, robotZ / mmPerInch);
            this.myOpMode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Yaw/Heading} = %.0f, %.0f, %.0f", robotAngleX, robotAngleY, robotAngleZ);


            myOpMode.telemetry.addData("Robot", "[X]:[Y] (B) [%5.0fmm]:[%5.0fmm] (%4.0f°)",
                    robotX, robotY, robotBearing);
            myOpMode.telemetry.addData("Target", "[R] (B):(RB) [%5.0fmm] (%4.0f°):(%4.0f°)",
                    targetRange, targetBearing, relativeBearing);

            myOpMode.telemetry.addData("- Turn    ", "%s %4.0f°", relativeBearing < 0 ? ">>> CW " : "<<< CCW", Math.abs(relativeBearing));
            myOpMode.telemetry.addData("- Strafe  ", "%s %5.0fmm", robotY < 0 ? "LEFT" : "RIGHT", Math.abs(robotY));
            myOpMode.telemetry.addData("- Distance", "%5.0fmm", Math.abs(robotX));
        } else {
            myOpMode.telemetry.addData("Visible", "- - - -");
        }
    }

    public int detectSkystoneOnField(LinearOpMode myOpMode, boolean isBlue) {

        this.initVuforia(myOpMode);
        this.activateTracking();

        int result = 0;
        if (isBlue) {
            result = this.blueDetectSkystoneOnField();
        } else {
            result = this.redDetectSkystoneOnField();
        }

        this.deactivateTracking();
        return result;
    }


    private double[] detectSkystone() {

        double[] targetCoordinates = null;
        if (this.targetsAreVisible()) {
            String targetName = this.getTargetName();
            if (Constants.VISIBLE_TARGET_NAME.equals(targetName)) {
                //  RobotX, RobotY, RobotZ, Yaw, Hypotenuse, Theta, relativeBearing
                targetCoordinates = this.getTargetCoordinatesInInches();
            }
        }
        return targetCoordinates;
    }

    private int blueDetectSkystoneOnField() {

        int detectedStoneNumber = 3;
        while (!myOpMode.isStarted()) {

            double[] targetCoordinates = this.detectSkystone();
            if (targetCoordinates != null) {

                this.addNavTelemetry();

                double dY = targetCoordinates[1];

                //  Blue loading zone;
                double s1 = -5.3;
                double s2 = 1.2;
                double s3 = 9.3;

                double buffer = 1.5;

                if (dY < (s1+buffer)) {
                    myOpMode.telemetry.addData("Found stone 1 at ", dY);
                    detectedStoneNumber = 1;
                } else if ((s2-buffer) < dY && dY <= (s2+buffer)) {
                    myOpMode.telemetry.addData("Found stone 2 at ", dY);
                    detectedStoneNumber = 2;
                } else if ((s3-buffer) <= dY ) {
                    myOpMode.telemetry.addData("Found stone 3 at ", dY);
                    detectedStoneNumber = 3;
                } else {
                    myOpMode.telemetry.addData("Fix buffer!  Found unknown stone at ", dY);
                    detectedStoneNumber = 3;
                }

            } else {

                myOpMode.telemetry.addData("Skystone", " Searching.");

                //  Default is 3rd stone;
                detectedStoneNumber = 3;
            }

            myOpMode.telemetry.update();
            myOpMode.idle();
        }

        return detectedStoneNumber;
    }


    private int redDetectSkystoneOnField() {

        int detectedStoneNumber = 3;
        while (!myOpMode.isStarted()) {
            double[] targetCoordinates = this.detectSkystone();
            if (targetCoordinates != null) {

                this.addNavTelemetry();

                double dY = targetCoordinates[1];

                //  Blue loading zone;
                double s1 = 5.7;
                double s2 = -1.3;
                double s3 = -7.4;
                double buffer = 1.5;

                if ((s1-buffer) < dY) {
                    myOpMode.telemetry.addData("Found stone 1 at ", dY);
                    detectedStoneNumber = 1;
                } else if ((s2-buffer) < dY && dY <= (s2+buffer)) {
                    myOpMode.telemetry.addData("Found stone 2 at ", dY);
                    detectedStoneNumber = 2;
                } else if (dY <= (s3+buffer)) {
                    myOpMode.telemetry.addData("Found stone 3 at ", dY);
                    detectedStoneNumber = 3;
                } else {
                    myOpMode.telemetry.addData("Fix buffer!  Found unknown stone at ", dY);
                    detectedStoneNumber = 3;
                }

            } else {

                myOpMode.telemetry.addData("Skystone", " Searching.");

                //  Default is 3rd stone;
                detectedStoneNumber = 3;
            }

            myOpMode.telemetry.update();
            myOpMode.idle();
        }

        return detectedStoneNumber;
    }

}
