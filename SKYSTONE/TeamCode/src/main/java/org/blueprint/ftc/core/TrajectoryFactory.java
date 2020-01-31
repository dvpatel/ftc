package org.blueprint.ftc.core;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.roadrunner.util.AssetsTrajectoryLoader;

import java.io.IOException;
import java.util.HashMap;

/**
 * Helper class to load all trajectories for Skystone
 */
public class TrajectoryFactory {

    private static TrajectoryFactory tF ;

    private String[] trajectories =
            {"BlueFoundation", "BlueLoadingStone1", "BlueLoadingStone2", "BlueLoadingStone3",
                    "RedFoundation", "RedLoadingStone1", "RedLoadingStone2", "RedLoadingStone3"};

    private static HashMap<String, Trajectory> trajMap = new HashMap<String, Trajectory>();

    private TrajectoryFactory() throws IOException {
        this.setup();
    }

    public static TrajectoryFactory getInstance() throws IOException{

        if (tF == null) {
            tF = new TrajectoryFactory();
        }

        return tF;
    }

    private void setup() throws IOException {
        for(String t : trajectories) {
            trajMap.put(t, AssetsTrajectoryLoader.load(t)) ;
        }
    }

    public Trajectory getTrajectory(String name) {
        Trajectory t = trajMap.get(name);

        if (t == null) {
            throw new NullPointerException("Trajectory " + name + " not found.");
        }

        return t;
    }
}
