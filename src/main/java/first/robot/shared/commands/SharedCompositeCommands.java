package first.robot.shared.commands;

import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.shared.subsystems.climber.Climber;
import first.robot.shared.subsystems.climber.ClimberConstants.ClimberGoal;
import first.robot.shared.subsystems.intake.Intake;
import first.robot.shared.util.AllianceFlipUtil;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A class that holds composite commands, which are sequences of commands for complex robot actions.
 */
public class SharedCompositeCommands {

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
        .ignoringDisable(true);
  }

  public static Command deployClimber(Intake intake, Climber climber) {
    return Commands.sequence(
        intake.stow(),
        Commands.parallel(
            Commands.sequence(intake.stopCollect()),
            climber.setPositionGoal(ClimberGoal.L1_POSITION_GOAL.getPosition(), GainSlot.ZERO)));
  }

  public static Command unClimbPostAuto(Intake intake, Climber climber) {
    return Commands.parallel(
        intake.stow(), climber.setPositionGoal(ClimberGoal.UNCLIMB.getPosition(), GainSlot.ZERO));
  }

  public static Command updateCurrentLimits(
      SwerveDrive drive, double driveCurrentLimit, double turnCurrentLimit) {
    return Commands.runOnce(() -> drive.updateCurrentLimits(driveCurrentLimit, turnCurrentLimit))
        .ignoringDisable(true);
  }
}
