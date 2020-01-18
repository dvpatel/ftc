/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.blueprint.ftc.core.AbstractLinearOpMode;
import org.blueprint.ftc.core.Constants;
import org.blueprint.ftc.core.IntakeSystem;
import org.blueprint.ftc.core.SkystoneDetector;


//  See
//  https://github.com/gearsincorg/FTCVuforiaDemo/blob/master/TeleopOpmode.java

@TeleOp(name = "SkystoneTele")
@Disabled
public class SkystoneDetectionTele extends AbstractLinearOpMode {

    private SkystoneDetector skystoneDetector;
    private IntakeSystem intakeSystem;

    private static final double VELOCITY = 0.35 * Constants.MOTOR_MAX_VELOCITY;  // ticks per second
    double[] targetCoordinates;
    boolean foundStone;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void initOpMode() throws InterruptedException {
        this.initRosie();

        this.intakeSystem = this.rosie.getIntakeSystem();

        this.skystoneDetector = this.rosie.getSkystoneDetector();
        this.skystoneDetector.initVuforia(this);
        this.skystoneDetector.activateTracking();

        //  wait for camera to be activated;
        sleep(10000);

        //  detect during init;  give 30 seconds;
        runtime.reset();
        while ((runtime.seconds() < 30.0)) {
            foundStone = this.detectSkystone();
            if (!foundStone) {
                telemetry.addData("Skystone", " searching.");
                telemetry.addData("Time remaining", runtime);
                telemetry.update();
            } else {
                break;
            }

            //  this.skystoneDetector.addNavTelemetry();
            idle();
        }
        this.skystoneDetector.deactivateTracking();

        if (foundStone) {
            double dY = targetCoordinates[1];
            telemetry.addData("Skystone", " Found it.");
            telemetry.addData("dY", dY);
            telemetry.update();

            sleep(5000);
        }

    }

    @Override
    public void stopOpMode() {
        // Disable Tracking when we are done;
        this.skystoneDetector.deactivateTracking();
        this.intakeSystem.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        this.initOpMode();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Prompt User
            telemetry.addData(">", "Press start");

            // Display any Nav Targets while we wait for the match to start
            this.skystoneDetector.targetsAreVisible();
            this.skystoneDetector.addNavTelemetry();
            telemetry.update();
        }

        this.waitToPressStart();

        while(opModeIsActive()) {

            if (this.foundStone) {
                this.driveToTarget(this.targetCoordinates);

                //  this.intakeSystem.start();

                stop();
            }

        }

        this.stopOpMode();
    }

    private boolean detectSkystone() {
        if (skystoneDetector.targetsAreVisible()) {
            String targetName = skystoneDetector.getTargetName();
            if (Constants.VISIBLE_TARGET_NAME.equals(targetName)) {

                // robotBearing:  yaw / heading;
                //  targetRange:  Hypotenuse
                //  targetBearing:  Angle between x and y
                //  relativeBearing:  targetBearing-robotBearing
                // robotX, robotY, robotX, robotBearing, targetRange, targetBearing, relativeBearing;
                this.targetCoordinates = this.skystoneDetector.getTargetCoordinatesInInches();
                return true;
            }
        }
        return false;
    }

    private void driveToTarget(double[] targetCoordinates) {

        double dX = Math.abs(targetCoordinates[0]);
        double dY = targetCoordinates[1];
        double angle = targetCoordinates[5];
        double thetaRadians = Math.toRadians(90 - Math.abs(angle));
        double hypot = targetCoordinates[4];
        double totalY = hypot * Math.cos(thetaRadians);

        //  intake offset;
        double buffer = 2.0;
        double intakeOffset = 3.25;
        double robotWidth = Constants.DRIVETRAIN_WIDTH / 2;
        double stoneOffset = 1.0;


        double distX = dX;
        double distY = 0;
        if (dY > 0) {  //  right side of camera origin;
            distY = robotWidth + totalY+buffer;
        } else if (dY < 0) {  //  left side of camera origin
            distY = robotWidth - totalY + stoneOffset;
        }

        telemetry.addData("Angle", angle);
        telemetry.addData("TotalY", totalY);
        telemetry.addData("Y", dY);
        telemetry.update();

        //  direction;  Negative is left of robot;  Positive is right of robot
        //  positive velocity strafe left; negative strafe right
        if (dY > 0) {
            this.strafeRight(distY, VELOCITY);
        } else if (dY < 0) {
            this.strafeLeft(distY, VELOCITY);
        }

        this.drive(distX, VELOCITY);

    }
}
