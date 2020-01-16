package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.CRServoController;
import org.blueprint.ftc.core.GamepadDriver;
import org.blueprint.ftc.core.LiftSystem;
import org.blueprint.ftc.core.ServoController;
import org.blueprint.ftc.core.SimpleMotor;

@TeleOp(name = "LiftSystemTest")
//  @Disabled
public class LiftSystemTele extends AbstractLinearOpMode {

    private LiftSystem liftSystem;
    private static final int SLEEP_TIME = 2000;

    @Override
    public void initOpMode() throws InterruptedException {
        //  Make sure Rosie is initialized ;
        this.initRosie();
        this.liftSystem = this.rosie.getLiftSystem();

        telemetry.addData("Mode", "init complete;  Running");
        telemetry.update();
    }

    @Override
    public void stopOpMode() {
        this.liftSystem.reset();
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        this.initOpMode();

        GamepadDriver gpd = this.rosie.getGamepadDriver();

        //  Wait for start button ;
        this.waitToPressStart();

        //  Tells motor to run to target using position and power ;  Make sure t reset encoder when done!
        //  this.simpleMotor.setTargetPosition(2500);

        //  make sure power is between -1 and 1 ;
        telemetry.addData("LiftSystem:  ", "Pickup");
        telemetry.update();
        this.liftSystem.pickup();
        sleep(SLEEP_TIME);

        telemetry.addData("LiftSystem:  ", "Move back");
        this.liftSystem.lift(12);
        telemetry.update();
        sleep(SLEEP_TIME);

        telemetry.addData("LiftSystem:  ", "Move down");
        this.liftSystem.lift(-12);
        telemetry.update();
        sleep(SLEEP_TIME);

        telemetry.addData("LiftSystem:  ", "Release");
        this.liftSystem.releaseObject();
        telemetry.update();
        sleep(SLEEP_TIME);

        telemetry.addData("LifeSystem:  ", "Back to base");
        this.liftSystem.backToBase();
        telemetry.update();
        sleep(SLEEP_TIME);

        this.stopOpMode();
    }


}
