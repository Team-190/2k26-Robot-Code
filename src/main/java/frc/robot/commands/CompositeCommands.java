package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Consumer;
import java.util.function.Supplier;

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
    public static Command resetHeading(
        SwerveDrive drive,
        Consumer<Pose2d> resetHeadingConsumer,
        Supplier<Translation2d> currentRobotTranslation) {
      return Commands.runOnce(
              () -> {
                resetHeadingConsumer.accept(
                    new Pose2d(
                        currentRobotTranslation.get(), AllianceFlipUtil.apply(new Rotation2d())));
              })
          .beforeStarting(Commands.print("Resetting drive..."))
          .andThen(Commands.print("...Drive Reset"))
          .ignoringDisable(true);
    }
  }
}
