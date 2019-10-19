package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

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

    //  My pushbotrobot ;
    private Rosie rosie;

    private ColorSensorController colorSensor ;

    private View relativeLayout ;

    @Override
    public void initRobot() {

        this.rosie = new Rosie();
        this.rosie.init(hardwareMap);

        //  ColorSensor detector ;
        this.colorSensor = this.rosie.getColorSensorController();


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

    // loop and read the RGB and distance data.
    @Override
    public void runRobot() {

        // send the info back to driver station using telemetry function.
        //  telemetry.clearAll();
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", this.colorSensor.getDistance()));


        if (colorSensor.isTargetBlue() || colorSensor.isTargetRed()) {

            int[] argb = this.colorSensor.argb();

            telemetry.addData("Alpha", argb[Constants.COLOR_ALPHA]);
            telemetry.addData("TargetColor.Red ", argb[Constants.COLOR_RED]);
            telemetry.addData("TargetColor.Green ", argb[Constants.COLOR_GREEN]);
            telemetry.addData("TargetColor.Blue ", argb[Constants.COLOR_BLUE]);
        }

        telemetry.update();

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, colorSensor.getHSV()));
            }
        });


        //while (colorSensor.alpha() < 20) {
        // Drive Forward
        //}

    }
}
