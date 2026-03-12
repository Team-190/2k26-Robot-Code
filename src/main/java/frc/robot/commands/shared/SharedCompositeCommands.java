package frc.robot.commands.shared;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.subsystems.shared.climber.Climber;
import frc.robot.subsystems.shared.climber.ClimberConstants.ClimberGoal;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Consumer;
import java.util.function.Supplier;

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

  public static Command deployClimber(V1_DoomSpiralIntake intake, Climber climber) {
    return Commands.sequence(
        intake.stow(),
        Commands.parallel(
            Commands.sequence(
                intake.setRollerVoltage(-12.0),
                intake.waitUntilIntakeAtGoal(),
                intake.stopRoller()),
            climber.setPositionGoal(ClimberGoal.L1_POSITION_GOAL.getPosition(), GainSlot.ZERO)));
  }

  public static Command unClimbPostAuto(V1_DoomSpiralIntake intake, Climber climber) {
    return Commands.parallel(
        intake.stow(), climber.setPositionGoal(ClimberGoal.UNCLIMB.getPosition(), GainSlot.ZERO));
  }
}
