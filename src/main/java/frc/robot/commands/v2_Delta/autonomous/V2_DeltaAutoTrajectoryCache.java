package frc.robot.commands.v2_Delta.autonomous;

import choreo.Choreo;
import choreo.trajectory.Trajectory;

public class V2_DeltaAutoTrajectoryCache {
  public static final Trajectory<?> TURRET_TEST = Choreo.loadTrajectory("TURRET_TEST").get();

  public static final Trajectory<?> LEFT_OP_SAFE_1 = Choreo.loadTrajectory("LEFT_OP_SAFE_1").get();
  public static final Trajectory<?> LEFT_OP_2 = Choreo.loadTrajectory("LEFT_OP_2").get();
  public static final Trajectory<?> RIGHT_OP_SAFE_1 =
      Choreo.loadTrajectory("RIGHT_OP_SAFE_1").get();
  public static final Trajectory<?> RIGHT_OP_2 = Choreo.loadTrajectory("RIGHT_OP_2").get();
}
