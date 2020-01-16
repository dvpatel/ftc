package org.blueprint.ftc.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.blueprint.ftc.core.Constants;

public class TouchSensorController {

    private TouchSensor touch ;

    public TouchSensorController(HardwareMap hardwareMap, String deviceName) {
        this.touch = hardwareMap.touchSensor.get(deviceName);
    }

    public TouchSensor getSensor() {
        return this.touch ;
    }

    public boolean isPressed() {
        return this.touch.isPressed();
    }
}
