package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.CRServoController;
import org.blueprint.ftc.core.Constants;
import org.blueprint.ftc.core.GamepadDriver;
import org.blueprint.ftc.core.LiftSystem;
import org.blueprint.ftc.core.ServoController;
import org.blueprint.ftc.core.SimpleMotor;

import java.util.Random;

@TeleOp(name = "LiftSystemTest")
@Disabled
public class LiftSystemTele extends AbstractLinearOpMode {

    private LiftSystem liftSystem;
    private static final int SLEEP_TIME = 2000;

    private double randomHeight;

    @Override
    public void initOpMode() throws InterruptedException {
        //  Make sure Rosie is initialized ;
        this.initRosie();
        this.liftSystem = this.rosie.getLiftSystem();
        this.liftSystem.setLinearOpMode(this);

        this.randomHeight = this.randomDouble(0.5, (Constants.SIMPLE_WHEEL_MAX_DISTANCE * 0.95));
        telemetry.addData("Mode", "init complete;  Running");
    }

    @Override
    public void stopOpMode() {
        this.liftSystem.reset();
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        this.initOpMode();
        telemetry.update();

        GamepadDriver gpd = this.rosie.getGamepadDriver();

        //  Wait for start button ;
        this.waitToPressStart();

        telemetry.addData("LiftSystem:  ", this.randomHeight + " inches.");
        this.liftSystem.lift(this.randomHeight);
        telemetry.update();
        sleep(5000);

        //  make sure power is between -1 and 1 ;
        telemetry.addData("LiftSystem:  ", "Pickup");
        telemetry.update();
        this.liftSystem.pickup();
        sleep(5000);

        telemetry.addData("LifeSystem:  ", "Back to base");
        this.liftSystem.backToBase();
        telemetry.update();
        sleep(SLEEP_TIME);

        this.stopOpMode();
    }


    double randomDouble(double min, double max)
    {
        Random rand = new Random();
        return rand.nextDouble()*(max-min) + min;
    }



}
