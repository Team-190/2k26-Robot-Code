package frc.robot.commands.v2_Delta.autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.subsystems.v2_Delta.V2_DeltaRobotState;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooter;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooterConstants.ShooterGoal;
import frc.robot.util.BetterAutoChooser;

public class V2_TurretTestAuto {

  public static final BetterAutoChooser.AutoRoutineConfiguration getAutoRoutine(
      SwerveDrive drive, V2_DeltaShooter shooter) {
    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("TURRET TEST");

    AutoTrajectory TURRET_TEST = routine.trajectory(V2_DeltaAutoTrajectoryCache.TURRET_TEST);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                TURRET_TEST.resetOdometry(),
                Commands.parallel(TURRET_TEST.cmd(), shooter.setGoal(ShooterGoal.SCORE)),
                Commands.runOnce(() -> drive.stop())));

    return new BetterAutoChooser.AutoRoutineConfiguration(
        () -> routine,
        () -> TURRET_TEST.getInitialPose().orElse(new Pose2d()),
        () ->
            Commands.runOnce(
                () -> {
                  drive.setAutoControllers(
                      Gains.builder()
                          .withKP(new LoggedTunableNumber("Drive/Auto/Climb/Translation Kp", 5.0))
                          .withKD(new LoggedTunableNumber("Drive/Auto/Climb/Translation Kd", 0.0))
                          .build(),
                      Gains.builder()
                          .withKP(new LoggedTunableNumber("Drive/Auto/Climb/Rotation Kp", 5.0))
                          .withKD(new LoggedTunableNumber("Drive/Auto/Climb/Rotation Kd", 0.0))
                          .build());
                  V2_DeltaRobotState.setAutoTrajectory(TURRET_TEST);
                }));
  }
}
