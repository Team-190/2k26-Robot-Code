package frc.robot.commands.v1_Gamma.autonomous;

import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.subsystems.v1_Gamma.V1_GammaRobotState;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;

public class V1_GammaAutonomousTest {
  public static Command TEST_PATH_CMD;

  public static Command testAuto(SwerveDrive drive) {
    Optional<? extends Trajectory<?>> TEST_PATH =
        drive.getAutoFactory().cache().loadTrajectory("TEST_PATH");

    return Commands.sequence(
        Commands.runOnce(
            () ->
                V1_GammaRobotState.resetPose(
                    TEST_PATH.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        TEST_PATH_CMD);
  }
}
