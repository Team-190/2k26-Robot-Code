package frc.robot.util;

import choreo.Choreo.TrajectoryCache;
import choreo.trajectory.Trajectory;
import frc.robot.RobotConfig;

/** Creates a cache to store trajectories. */
public class PathCache {

  private static TrajectoryCache cache;

  /** Creates the cache and loads trajectories */
  static {
    cache = new TrajectoryCache();

    switch (RobotConfig.ROBOT) {
      case V0_FUNKY:
      case V0_FUNKY_SIM:
        break;

      case V1_GAMMA:
      case V1_GAMMA_SIM:
        cache.loadTrajectory("DEPOT");
        cache.loadTrajectory("LEFT_TRENCH");
        cache.loadTrajectory("RIGHT_TRENCH");
        cache.loadTrajectory("OUTPOST_1");
        cache.loadTrajectory("OUTPOST_2");
        cache.loadTrajectory("OUTPOST_DEPOT");
        break;
    }
  }

  /**
   * Returns the stated trajectory from the cache.
   *
   * @param name The name of the desired trajectory.
   */
  public static Trajectory<?> getTrajectory(String name) {
    return cache.loadTrajectory(name).get();
  }
}
