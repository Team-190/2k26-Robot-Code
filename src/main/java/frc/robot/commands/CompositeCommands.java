package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

/**
 * A class that holds composite commands, which are sequences of commands for complex robot actions.
 */
public class CompositeCommands {
  /** A class that holds composite commands that are shared across different robot versions. */
  public static final class SharedCommands {
    /**
     * Creates a command to reset the robot's heading to the alliance-specific zero.
     *
     * @param drive The drive subsystem.
     * @return A command to reset the heading.
     */
    public static Command resetHeading(Drive drive) {
      return Commands.runOnce(
              () -> {
                RobotState.resetRobotPose(
                    new Pose2d(
                        RobotState.getRobotPoseField().getTranslation(),
                        AllianceFlipUtil.apply(new Rotation2d())));
              })
          .ignoringDisable(true);
    }
  }
}
