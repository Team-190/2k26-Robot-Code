package frc.robot.commands.v2_Delta.autonomous;

import choreo.Choreo;
import choreo.trajectory.Trajectory;

public class V2_DeltaAutoTrajectoryCache {
  public static final Trajectory<?> TURRET_TEST = Choreo.loadTrajectory("TURRET_TEST").get();
  public static final Trajectory<?> OP = Choreo.loadTrajectory("OP").get();
}
