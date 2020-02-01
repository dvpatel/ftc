package org.blueprint.ftc.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Module to pickup stones using stone arm / gripper
 */
public class StoneSystem {

    private ServoController stoneArm;
    private ServoController stoneGripper;

    private LinearOpMode myOpMode;

    public StoneSystem(HardwareMap hardwareMap) {
        this.stoneArm = new ServoController(hardwareMap, Constants.STONE_ARM);
        //  this.stoneGripper = new ServoController(hardwareMap, Constants.STONE_GRIPPER, true);
        this.stoneGripper = new ServoController(hardwareMap, Constants.STONE_GRIPPER);
    }

    public void setLinearOpMode(LinearOpMode myOpMode) {
        this.myOpMode = myOpMode;
    }

    private double calculatePosition(double degrees) {
        return Range.clip(degrees, 0, 270) / 270;
    }


    /**
     * Position system for stone pickup
     */
    public void positionSystem() {
        //  bring down arm; open gripper
        this.openGripper();
        this.bringDownArm();
    }

    /**
     * Reset stone system position;
     */
    public void pickupStone() {
        //  bring down arm; open gripper
        this.closeGripper();
        this.myOpMode.sleep(500);
        this.bringUpArmBelowBridge();
    }


    public void putdownStone() {
        //  bring down arm; open gripper
        this.bringDownArm();
        this.myOpMode.sleep(500);
        this.openGripper();
    }


    /**
     * Bring down stone system arm
     */
    public void bringDownArm() {
        this.stoneArm.setPosition(0.0);
    }

    /**
     * Bring up stone system arm
     */
    public void bringUpArm() {
        this.stoneArm.setPosition(0.60);  //  Was 0.60
    }

    public void bringUpArmBelowBridge() {
        this.stoneArm.setPosition(0.22);  //  Was 0.60
    }


    /**
     * Open gripper
     */
    public void openGripper() {
        this.stoneGripper.setPositionByDegrees(0);
    }

    /**
     * close stone system gripper
     */
    public void closeGripper() {
        this.stoneGripper.setPositionByDegrees(110);
    }

}
