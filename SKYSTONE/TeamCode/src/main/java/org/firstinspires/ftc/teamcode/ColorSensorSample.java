package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Locale;


/**
 * DP
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="BasicColorTest", group="Linear Opmode")
@Disabled
public class ColorSensorSample extends BaseLinearOpMode {

    private ColorSensorController colorSensor ;

    private View relativeLayout ;

    @Override
    public void initRobot() {
        // get a reference to the color sensor.
        this.colorSensor = new ColorSensorController(hardwareMap) ;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        this.relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    }

    @Override
    public void stopRobot() {
        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }

    @Override
    public void runRobot() {
        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        final float[] hsvValues = this.colorSensor.RGBToHSV();
        int[] argb = this.colorSensor.argb() ;

        // send the info back to driver station using telemetry function.
        //  telemetry.clearAll();
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", this.colorSensor.getDistance()));
        telemetry.addData("Alpha", argb[Constants.COLOR_ALPHA]);


        if (this.isTargetColor(hsvValues))
        {
            telemetry.addData("TargetColor.Red ", argb[Constants.COLOR_RED]);
            telemetry.addData("TargetColor.Green ", argb[Constants.COLOR_GREEN]);
            telemetry.addData("TargetColor.Blue ", argb[Constants.COLOR_BLUE]);
        }

        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.update();

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, hsvValues));
            }
        });


        //while (colorSensor.alpha() < 20) {
        // Drive Forward
        //}

    }

    private boolean isTargetColor(float... hsvValues) {

        //  Hue:  Red:  340 - 20 ;
        //  Saturation >= 0.6

        // Hue: Blue:  210 - 275;
        //  Saturation >= 0.6

        //  H:  0 - 360;  Color range
        //  S:  0 - 1;  Color Intensity;  0
        //  V:  0 - 1; brightness of color ;

        float hue = hsvValues[0] ;
        float sat = hsvValues[1] ;

        //  return (sat > 0.6 && (hue > 340 && hue < 20)) ? true : false ; // red
        //  return (sat > 0.6 && (hue > 200 && hue < 275)) ? true : false ; //  blue

        return sat > Constants.TARGET_COLOR_SATURATION &&
                (hue > Constants.TARGET_COLOR_HUE_LOW && hue < Constants.TARGET_COLOR_HUE_HIGH);
    }
}
