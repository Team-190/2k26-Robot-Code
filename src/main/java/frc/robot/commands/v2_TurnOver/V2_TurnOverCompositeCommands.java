package frc.robot.commands.v2_TurnOver;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import frc.robot.subsystems.shared.climber.Climber;
import frc.robot.subsystems.shared.climber.ClimberConstants.ClimberGoal;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.v2_TurnOver.V2_TurnOverRobotState;
import frc.robot.subsystems.v2_TurnOver.clopper.V2_TurnOverClopper;
import frc.robot.subsystems.v2_TurnOver.shooter.V2_TurnOverShooter;
import frc.robot.subsystems.v2_TurnOver.shooter.V2_TurnOverShooterConstants.ShooterGoal;
import frc.robot.subsystems.v2_TurnOver.shooter.V2_TurnOverShotCalculator;
import frc.robot.util.command.ContinuousConditionalCommand;
import org.littletonrobotics.junction.Logger;

public class V2_TurnOverCompositeCommands {

  public static Command hold(V2_TurnOverClopper clopper, V2_TurnOverShooter shooter) {
    return Commands.parallel(
        clopper.stopBallTunnel(),
        clopper.stopRollerFloor(),
        clopper.idle(),
        shooter.setGoal(() -> ShooterGoal.STOW),
        Commands.runOnce(V2_TurnOverShotCalculator::clear));
  }

  public static Command holdAndIntake(
      V2_TurnOverClopper clopper, V2_TurnOverShooter shooter, Intake intake) {
    return Commands.parallel(
        clopper.stopBallTunnel(),
        clopper.stopRollerFloor(),
        clopper.idle(),
        Commands.sequence(shooter.setGoal(ShooterGoal.IDLE), shooter.stopAll()),
        intake.setLinkageVoltage(0),
        intake.setOverrideRollerVoltage(8),
        Commands.runOnce(V2_TurnOverShotCalculator::clear));
  }

  private static boolean toggleShouldHold = false;

  public static Command toggleHold(V2_TurnOverClopper clopper, V2_TurnOverShooter shooter) {
    return Commands.either(
            hold(clopper, shooter), scoreOrFeedCommand(shooter, clopper), () -> toggleShouldHold)
        .beforeStarting(
            () -> {
              toggleShouldHold = !toggleShouldHold;
              Logger.recordOutput("Toggle hold", toggleShouldHold);
            });
  }

  public static Command scoreOrFeedCommand(V2_TurnOverShooter shooter, V2_TurnOverClopper clopper) {
    return Commands.parallel(
        shooter.setGoal(
            () -> V2_TurnOverRobotState.isInAllianceZone() ? ShooterGoal.SCORE : ShooterGoal.FEED),
        runHopperWhenReady(shooter, clopper));
  }

  public static Command fixedShotCommand(
      V2_TurnOverShooter shooter,
      V2_TurnOverClopper clopper,
      V2_TurnOverRobotState.FixedShots fixedShot) {
    return Commands.parallel(
        Commands.runOnce(V2_TurnOverShotCalculator::clear),
        shooter.runFixedShot(fixedShot),
        runHopperWhenReady(shooter, clopper));
  }

  public static Command runHopperWhenReady(V2_TurnOverShooter shooter, V2_TurnOverClopper clopper) {
    return new ContinuousConditionalCommand(
        clopper.feedShooterRollerFloor().alongWith(clopper.feedShooterBallTunnel(), clopper.idle()),
        new ContinuousConditionalCommand(
            clopper
                .setOverrideBallsToWallVoltage(Volts.of(-5))
                .alongWith(clopper.stopBallTunnel(), clopper.stopRollerFloor()),
            clopper.stopBallTunnel().alongWith(clopper.stopRollerFloor(), clopper.idle()),
            () -> !V2_TurnOverRobotState.isProhibitShot() && !shooter.atGoal()),
        () -> !V2_TurnOverRobotState.isProhibitShot() && shooter.atGoal());
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
