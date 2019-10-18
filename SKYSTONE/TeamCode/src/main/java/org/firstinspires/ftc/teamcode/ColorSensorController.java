package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//  Color and Range Sensor Controller ;
public class ColorSensorController {

    private DistanceSensor sensorColorRange;
    private ColorSensor colorSensor;

    private float hsvValues[] = {0F, 0F, 0F};

    public ColorSensorController(HardwareMap hardwareMap) {

        // get a reference to the color sensor.
        this.colorSensor = hardwareMap.get(ColorSensor.class, Constants.COLOR_SENSOR_NAME);

        // get a reference to the distance sensor that shares the same name.
        this.sensorColorRange = hardwareMap.get(DistanceSensor.class, Constants.COLOR_SENSOR_NAME);

        // values is a reference to the hsvValues array.
        final float values[] = this.hsvValues;
    }

    public float[] RGBToHSV() {
        Color.RGBToHSV((int) (colorSensor.red() * Constants.SCALE_FACTOR),
                (int) (colorSensor.green() * Constants.SCALE_FACTOR),
                (int) (colorSensor.blue() * Constants.SCALE_FACTOR),
                this.hsvValues);

        return this.hsvValues ;
    }

    //  Return Alpha-Red-Green-Blue as int array ;
    public int[] argb(){
        int[] argb = {this.colorSensor.alpha(), this.colorSensor.red(), this.colorSensor.green(), this.colorSensor.blue() } ;
        return argb ;
    }

    public double getDistance() {
        return this.sensorColorRange.getDistance(DistanceUnit.CM) ;
    }

}
