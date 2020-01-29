package org.roadrunner.util;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.fasterxml.jackson.module.kotlin.KotlinModule;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.roadrunner.DriveConstants;

import java.io.IOException;
import java.io.InputStream;

/**
 * Set of utilities for loading trajectories from assets (the plugin save location).
 */
public class AssetsTrajectoryLoader {
    private static final ObjectMapper MAPPER = new ObjectMapper(new YAMLFactory());

    static {
        MAPPER.registerModule(new KotlinModule());
    }

    /**
     * Loads a trajectory config with the given name.
     */
    public static TrajectoryConfig loadConfig(String name) throws IOException {
        InputStream inputStream = AppUtil.getDefContext().getAssets().open("trajectory/" + name + ".yaml");
        return MAPPER.readValue(inputStream, TrajectoryConfig.class);
    }

    /**
     * Loads a trajectory with the given name.
     *
     * @see #loadConfig(String)
     */
    public static Trajectory load(String name) throws IOException {
        TrajectoryGroupConfig trajectoryGroupConfig = new TrajectoryGroupConfig(
                DriveConstants.BASE_CONSTRAINTS,
                TrajectoryGroupConfig.DistanceUnit.INCH,
                TrajectoryGroupConfig.DriveType.MECANUM,
                DriveConstants.TRACK_WIDTH,
                DriveConstants.WHEEL_BASE,
                DriveConstants.LATERAL_MULTIPLIER);

        return loadConfig(name).toTrajectory(trajectoryGroupConfig);
    }
}
