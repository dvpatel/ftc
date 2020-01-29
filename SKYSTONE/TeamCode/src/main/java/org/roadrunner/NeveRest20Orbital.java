package org.roadrunner;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFPositionParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

//======================================================================================
    /*  From MaxVelocityTest
    MotorName: NeveRest 20 Orbital Gearmotor
    TicksPerRev: 537.6
    achMaxRPMFrac: 0.85
    achMaxTPS: 2589.44

    Default Position Coeffs for Orbital 20:
      [PIDFCoefficients(p=10.000000 i=0.049988 d=0.000000 f=0.000000 alg=LegacyPID)]
    Default Velocity Coeffs for Orbital 20:
      [PIDFCoefficients(p=10.000000 i=3.000000 d=0.000000 f=0.000000 alg=LegacyPID)]
     */

//  maxCPS = ticksPerRev*maxRPM/60 or 3046.4
//  Is Orientation correct?

@MotorType(ticksPerRev = 537.6, gearing = 19.2, maxRPM = 340, orientation = Rotation.CW)
@DeviceProperties(xmlTag = "NeveRest20Orbital", name = "NeveRest 20 Orbital Gearmotor", builtIn = true)
@DistributorInfo(distributor = "AndyMark", model = "am-3637", url = "https://www.andymark.com/products/neverest-orbital-20-gearmotor")
@ExpansionHubPIDFVelocityParams(P=10.0, I=3.0, F=0)
@ExpansionHubPIDFPositionParams(P=10.0)
public interface NeveRest20Orbital {
}