package org.blueprint.ftc.core;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//  Color and Range Sensor Controller ;
public class ColorSensorController {

    private DistanceSensor sensorColorRange;
    private ColorSensor colorSensor;

    private float[] hsvValues = {0F, 0F, 0F};

    public ColorSensorController(HardwareMap hardwareMap) {

        // get a reference to the color sensor.
        this.colorSensor = hardwareMap.get(ColorSensor.class, Constants.COLOR_SENSOR_NAME);

        // get a reference to the distance sensor that shares the same name.
        this.sensorColorRange = hardwareMap.get(DistanceSensor.class, Constants.COLOR_SENSOR_NAME);

        // values is a reference to the hsvValues array.
        final float[] values = this.hsvValues;
    }

    private float[] RGBToHSV() {
        Color.RGBToHSV((int) (colorSensor.red() * Constants.SCALE_FACTOR),
                (int) (colorSensor.green() * Constants.SCALE_FACTOR),
                (int) (colorSensor.blue() * Constants.SCALE_FACTOR),
                this.hsvValues);

        return this.hsvValues ;
    }

    public float[] getHSV() {
        return this.RGBToHSV();
    }

    //  Return Alpha-Red-Green-Blue as int array ;
    public int[] argb(){
        return new int[]{this.colorSensor.alpha(), this.colorSensor.red(), this.colorSensor.green(), this.colorSensor.blue()};
    }

    public boolean isTargetBlue() {

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        float[] hsvValues = this.RGBToHSV();

        //  Hue:  Red:  340 - 20 ;
        //  Saturation >= 0.6

        // Hue: Blue:  210 - 275;
        //  Saturation >= 0.6

        //  H:  0 - 360;  Color range
        //  S:  0 - 1;  Color Intensity;  0
        //  V:  0 - 1; brightness of color ;

        float hue = hsvValues[0];
        float sat = hsvValues[1];

        //  return (sat > 0.6 && (hue > 340 && hue < 20)) ? true : false ; // red
        //  return (sat > 0.6 && (hue > 200 && hue < 275)) ? true : false ; //  blue

        return sat > Constants.TARGET_COLOR_SATURATION &&
                (hue >= Constants.TARGET_COLOR_BLUE_HUE_LOW && hue <= Constants.TARGET_COLOR_BLUE_HUE_HIGH);
    }

    public boolean isTargetRed() {

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        float[] hsvValues = this.RGBToHSV();

        //  Hue:  Red:  340 - 20 ;
        //  Saturation >= 0.6

        //  H:  0 - 360;  Color range
        //  S:  0 - 1;  Color Intensity;  0
        //  V:  0 - 1; brightness of color ;

        float hue = hsvValues[0];
        float sat = hsvValues[1];

        //  return (sat > 0.6 && (hue > 340 && hue < 20)) ? true : false ; // red
        //  return (sat > 0.6 && (hue > 200 && hue < 275)) ? true : false ; //  blue

        return sat > Constants.TARGET_COLOR_SATURATION &&
                (hue > Constants.TARGET_COLOR_RED_HUE_LOW || hue < Constants.TARGET_COLOR_RED_HUE_HIGH);
    }

    public void ledOn() {
        this.colorSensor.enableLed(true);
    }

    public void ledOff() {
        this.colorSensor.enableLed(false);
    }

    public double getDistance() {
        return this.sensorColorRange.getDistance(DistanceUnit.CM) ;
    }


}
