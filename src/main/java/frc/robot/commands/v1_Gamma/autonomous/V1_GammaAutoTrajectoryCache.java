package frc.robot.commands.v1_Gamma.autonomous;

import choreo.Choreo;
import choreo.trajectory.Trajectory;

public class V1_GammaAutoTrajectoryCache {
    public static final Trajectory<?> DEPOT = Choreo.loadTrajectory("DEPOT").get();
    public static final Trajectory<?> LEFT_TRENCH = Choreo.loadTrajectory("LEFT_TRENCH").get();
    public static final Trajectory<?> RIGHT_TRENCH = Choreo.loadTrajectory("RIGHT_TRENCH").get();
    public static final Trajectory<?> OUTPOST_1 = Choreo.loadTrajectory("OUTPOST_1").get();
    public static final Trajectory<?> OUTPOST_2 = Choreo.loadTrajectory("OUTPOST_2").get();
    public static final Trajectory<?> OUTPOST_DEPOT = Choreo.loadTrajectory("OUTPOST_DEPOT").get();
}
