package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class TouchSensorController {

    private TouchSensor touch ;

    public TouchSensorController(HardwareMap hardwareMap) {
        this(hardwareMap, Constants.TOUCH_SENSOR_NAME) ;
    }

    public TouchSensorController(HardwareMap hardwareMap, String deviceName) {
        this.touch = hardwareMap.touchSensor.get(deviceName);
    }

    public TouchSensor getSensor() {
        return this.touch ;
    }

    public boolean pressed() {
        return this.touch.isPressed();
    }
}
