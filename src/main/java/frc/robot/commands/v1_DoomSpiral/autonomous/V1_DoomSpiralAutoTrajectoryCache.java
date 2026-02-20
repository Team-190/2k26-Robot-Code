package frc.robot.commands.v1_DoomSpiral.autonomous;

import choreo.Choreo;
import choreo.trajectory.Trajectory;

public class V1_DoomSpiralAutoTrajectoryCache {
  public static final Trajectory<?> DEPOT = Choreo.loadTrajectory("DEPOT.traj").get();
  public static final Trajectory<?> LEFT_TRENCH = Choreo.loadTrajectory("LEFT_TRENCH.traj").get();
  public static final Trajectory<?> RIGHT_TRENCH = Choreo.loadTrajectory("RIGHT_TRENCH.traj").get();
  public static final Trajectory<?> OUTPOST_1 = Choreo.loadTrajectory("OUTPOST_1.traj").get();
  public static final Trajectory<?> OUTPOST_2 = Choreo.loadTrajectory("OUTPOST_2.traj").get();
  public static final Trajectory<?> OUTPOST_DEPOT =
      Choreo.loadTrajectory("OUTPOST_DEPOT.traj").get();
}
