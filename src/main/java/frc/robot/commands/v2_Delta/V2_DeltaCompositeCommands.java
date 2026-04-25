package frc.robot.commands.v2_Delta;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import frc.robot.subsystems.shared.climber.Climber;
import frc.robot.subsystems.shared.climber.ClimberConstants.ClimberGoal;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.v2_Delta.V2_DeltaRobotState;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopper;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooter;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooterConstants.ShooterGoal;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShotCalculator;
import frc.robot.util.command.ContinuousConditionalCommand;
import org.littletonrobotics.junction.Logger;

public class V2_DeltaCompositeCommands {

  public static Command hold(V2_DeltaClopper clopper, V2_DeltaShooter shooter) {
    return Commands.parallel(
        clopper.stopBallTunnel(),
        clopper.stopRollerFloor(),
        clopper.idle(),
        shooter.setGoal(() -> ShooterGoal.STOW),
        Commands.runOnce(V2_DeltaShotCalculator::clear));
  }

  private static boolean toggleShouldHold = false;

  public static Command toggleHold(V2_DeltaClopper clopper, V2_DeltaShooter shooter) {
    return Commands.either(
            hold(clopper, shooter), scoreOrFeedCommand(shooter, clopper), () -> toggleShouldHold)
        .beforeStarting(
            () -> {
              toggleShouldHold = !toggleShouldHold;
              Logger.recordOutput("Toggle hold", toggleShouldHold);
            });
  }

  public static Command scoreOrFeedCommand(V2_DeltaShooter shooter, V2_DeltaClopper clopper) {
    return Commands.parallel(
        shooter.setGoal(
            () -> V2_DeltaRobotState.isInAllianceZone() ? ShooterGoal.SCORE : ShooterGoal.FEED),
        runHopperWhenReady(shooter, clopper));
  }

  public static Command fixedShotCommand(
      V2_DeltaShooter shooter, V2_DeltaClopper clopper, V2_DeltaRobotState.FixedShots fixedShot) {
    return Commands.parallel(
        Commands.runOnce(V2_DeltaShotCalculator::clear),
        shooter.runFixedShot(fixedShot),
        runHopperWhenReady(shooter, clopper));
  }

  public static Command runHopperWhenReady(V2_DeltaShooter shooter, V2_DeltaClopper clopper) {
    return new ContinuousConditionalCommand(
        clopper.feedShooterRollerFloor().alongWith(clopper.feedShooterBallTunnel(), clopper.idle()),
        new ContinuousConditionalCommand(
            clopper
                .setOverrideBallsToWallVoltage(Volts.of(-5))
                .alongWith(clopper.stopBallTunnel(), clopper.stopRollerFloor()),
            clopper.stopBallTunnel().alongWith(clopper.stopRollerFloor(), clopper.idle()),
            () -> !V2_DeltaRobotState.isProhibitShot() && !shooter.atGoal()),
        () -> !V2_DeltaRobotState.isProhibitShot() && shooter.atGoal());
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
