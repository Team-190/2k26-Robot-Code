package frc.robot.commands.v2_Delta;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.subsystems.shared.climber.Climber;
import frc.robot.subsystems.shared.climber.ClimberConstants.ClimberGoal;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooterConstants.HoodGoal;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopper;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopperConstants;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooter;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooterConstants.ShooterGoal;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.subsystems.v2_Delta.V2_DeltaRobotState;
import frc.robot.subsystems.v2_Delta.V2_DeltaConstants;
import frc.robot.subsystems.v2_Delta.V2_DeltaRobotState.FixedShotParameters;


public class V2_DeltaCompositeCommands {

    public static Command hold(V2_DeltaClopper clopper, V2_DeltaShooter shooter){
        return Commands.parallel(clopper.stopRollerFloor(), shooter.setGoal(ShooterGoal.STOW));
    }
    
    public static Command feedCommand(
      V2_DeltaShooter shooter, V2_DeltaClopper clopper) {
        return shooter
        .setGoal(ShooterGoal.FEED)
        .until(shooter::atGoal)
        .andThen(Commands.parallel(clopper.feedShooterBallTunnel(), clopper.feedShooterRollerFloor()));
  }

  public static Command scoreCommand(
      V2_DeltaShooter shooter, Intake intake, V2_DeltaClopper clopper) {
    return Commands.parallel(
        intake.stopRoller(),
        shooter.setGoal(ShooterGoal.SCORE));
  }

  public static Command stopShooterCommand(
      V2_DeltaShooter shooter, V2_DeltaClopper clopper) {
    return Commands.parallel(
        shooter.setGoal(ShooterGoal.STOW), clopper.stopBallTunnel(), clopper.stopRollerFloor());
  }

  public static Command fixedShotCommand(
      SwerveDrive drive,
      V2_DeltaShooter shooter,
      V2_DeltaClopper clopper,
      Intake intake,
      FixedShotParameters shotParameters) {
    return Commands.sequence(
        intake.stopRoller(),
        Commands.parallel(
                DriveCommands.rotateToAngle(
                    drive,
                    V2_DeltaConstants.DRIVE_CONSTANTS,
                    V2_DeltaRobotState::getHeading,
                    () -> AllianceFlipUtil.apply(shotParameters.robotAngle())),
            
                shooter.setGoal(ShooterGoal.SCORE)));
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
