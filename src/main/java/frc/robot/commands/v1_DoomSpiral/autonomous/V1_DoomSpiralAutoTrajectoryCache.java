package frc.robot.commands.v1_DoomSpiral.autonomous;

import choreo.Choreo;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class V1_DoomSpiralAutoTrajectoryCache {
  public static final Trajectory<?> DEPOT_AND_BACK_HUB_PATH_2 =
      Choreo.loadTrajectory("DEPOT_AND_BACK_HUB_PATH_2").get();
  public static final Trajectory<?> DEPOT_AND_CLIMB_PATH_2 =
      Choreo.loadTrajectory("DEPOT_AND_CLIMB_PATH_2").get();
  public static final Trajectory<?> DEPOT_AND_SIDE_WALL_PATH_2 =
      Choreo.loadTrajectory("DEPOT_AND_SIDE_WALL_PATH_2").get();
  public static final Trajectory<?> DEPOT_AND_XXX_PATH_1 =
      Choreo.loadTrajectory("DEPOT_AND_XXX_PATH_1").get();
  public static final Trajectory<?> LEFT_TRENCH_SIMPLE =
      Choreo.loadTrajectory("LEFT_TRENCH_SIMPLE").get();
  public static final Trajectory<?> RIGHT_TRENCH_SIMPLE =
      Choreo.loadTrajectory("RIGHT_TRENCH_SIMPLE").get();
  public static final Trajectory<?> TO_TEST_EFFECTIVENESS =
      Choreo.loadTrajectory("TO_TEST_EFFECTIVENESS").get();
  public static final Trajectory<?> CLIMB = Choreo.loadTrajectory("CLIMB").get();
  public static final Trajectory<?> LEFT_TRENCH_ANTI_BUCKS =
      Choreo.loadTrajectory("LEFT_TRENCH_ANTI_BUCKS").get();
  public static final Trajectory<?> RIGHT_TRENCH_ANTI_BUCKS =
      Choreo.loadTrajectory("RIGHT_TRENCH_ANTI_BUCKS").get();
  public static final Trajectory<?> LEFT_TRENCH_ANTI_BUCKS_CROSSES =
      Choreo.loadTrajectory("LEFT_TRENCH_ANTI_BUCKS_CROSSES").get();
  public static final Trajectory<?> RIGHT_TRENCH_ANTI_BUCKS_CROSSES =
      Choreo.loadTrajectory("RIGHT_TRENCH_ANTI_BUCKS_CROSSES").get();
  public static final Trajectory<?> LEFT_TRENCH_SIMPLE_CROSSES =
      Choreo.loadTrajectory("LEFT_TRENCH_SIMPLE_CROSSES").get();
  public static final Trajectory<?> RIGHT_TRENCH_SIMPLE_CROSSES =
      Choreo.loadTrajectory("RIGHT_TRENCH_SIMPLE_CROSSES").get();
  public static final Trajectory<?> LEFT_RETURN_TO_MID =
      Choreo.loadTrajectory("LEFT_RETURN_TO_MID").get();
  public static final Trajectory<?> RIGHT_RETURN_TO_MID =
      Choreo.loadTrajectory("RIGHT_RETURN_TO_MID").get();

  public static final Trigger GO_BACK_TRIGGER =
      RobotModeTriggers.autonomous().debounce(17.5, Debouncer.DebounceType.kRising);
}
