package frc.robot.commands.v2_Turnover;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import frc.robot.subsystems.shared.climber.Climber;
import frc.robot.subsystems.shared.climber.ClimberConstants.ClimberGoal;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.v2_Turnover.V2_TurnoverRobotState;
import frc.robot.subsystems.v2_Turnover.clopper.V2_TurnoverClopper;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverShooter;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverShooterConstants.ShooterGoal;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverShotCalculator;
import frc.robot.util.command.ContinuousConditionalCommand;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class V2_TurnoverCompositeCommands {

  public static Command hold(V2_TurnoverClopper clopper, V2_TurnoverShooter shooter) {
    return Commands.parallel(
        clopper.stopBallTunnel(),
        clopper.stopRollerFloor(),
        clopper.idle(),
        shooter.setGoal(() -> ShooterGoal.STOW),
        Commands.runOnce(V2_TurnoverShotCalculator::clear));
  }

  public static Command holdAndIntake(
      V2_TurnoverClopper clopper, V2_TurnoverShooter shooter, Intake intake) {
    return Commands.parallel(
        clopper.stopBallTunnel(),
        clopper.stopRollerFloor(),
        clopper.idle(),
        Commands.sequence(shooter.setGoal(ShooterGoal.IDLE), shooter.stopAll()),
        intake.setLinkageVoltage(0),
        intake.setOverrideRollerVoltage(8),
        Commands.runOnce(V2_TurnoverShotCalculator::clear));
  }

  private static boolean toggleShouldHold = false;

  public static Command toggleHold(
      V2_TurnoverClopper clopper, V2_TurnoverShooter shooter, BooleanSupplier invert) {
    return Commands.either(
            hold(clopper, shooter),
            scoreOrFeedCommand(shooter, clopper, invert),
            () -> toggleShouldHold)
        .beforeStarting(
            () -> {
              toggleShouldHold = !toggleShouldHold;
              Logger.recordOutput("Toggle hold", toggleShouldHold);
            });
  }

  public static Command scoreOrFeedCommand(
      V2_TurnoverShooter shooter, V2_TurnoverClopper clopper, BooleanSupplier invert) {
    return Commands.parallel(
        shooter.setGoal(
            () ->
                invert.getAsBoolean()
                    ? (V2_TurnoverRobotState.isInAllianceZone()
                        ? ShooterGoal.FEED
                        : ShooterGoal.SCORE)
                    : (V2_TurnoverRobotState.isInAllianceZone()
                        ? ShooterGoal.SCORE
                        : ShooterGoal.FEED)),
        runHopperWhenReady(shooter, clopper));
  }

  public static Command fixedShotCommand(
      V2_TurnoverShooter shooter,
      V2_TurnoverClopper clopper,
      V2_TurnoverRobotState.FixedShots fixedShot) {
    return Commands.parallel(
        Commands.runOnce(V2_TurnoverShotCalculator::clear),
        shooter.runFixedShot(fixedShot),
        runHopperWhenReady(shooter, clopper));
  }

  public static Command runHopperWhenReady(V2_TurnoverShooter shooter, V2_TurnoverClopper clopper) {
    return new ContinuousConditionalCommand(
        clopper.feedShooterRollerFloor().alongWith(clopper.feedShooterBallTunnel(), clopper.idle()),
        new ContinuousConditionalCommand(
            clopper
                .setOverrideBallsToWallVoltage(Volts.of(-5))
                .alongWith(clopper.stopBallTunnel(), clopper.stopRollerFloor()),
            clopper.stopBallTunnel().alongWith(clopper.stopRollerFloor(), clopper.idle()),
            () -> !V2_TurnoverRobotState.isProhibitShot() && !shooter.atGoal()),
        () -> !V2_TurnoverRobotState.isProhibitShot() && shooter.atGoal());
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
}
