package frc.robot.commands.v2_Delta.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.v2_Delta.V2_DeltaCompositeCommands;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.v2_Delta.V2_DeltaConstants;
import frc.robot.subsystems.v2_Delta.V2_DeltaRobotState;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopper;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooter;
import frc.robot.util.AllianceFlipUtil;

public class V2_DeltaAutoLeftOPBucks {
  public static Command getAutoRoutine(
      SwerveDrive drive, Intake intake, V2_DeltaClopper clopper, V2_DeltaShooter shooter) {

    PathPlannerPath OP_BUCKS_1;
    try {
      AutoBuilder.configure(
          V2_DeltaRobotState::getGlobalPose,
          V2_DeltaRobotState::resetPose,
          drive::getMeasuredChassisSpeeds,
          drive::runVelocity,
          new PPHolonomicDriveController(
              new PIDConstants(
                  V2_DeltaConstants.DRIVE_CONSTANTS.autoTranslationGains.kP().getAsDouble(),
                  V2_DeltaConstants.DRIVE_CONSTANTS.autoTranslationGains.kI().getAsDouble(),
                  V2_DeltaConstants.DRIVE_CONSTANTS.autoTranslationGains.kD().getAsDouble()),
              new PIDConstants(
                  V2_DeltaConstants.DRIVE_CONSTANTS.autoRotationGains.kP().getAsDouble(),
                  V2_DeltaConstants.DRIVE_CONSTANTS.autoRotationGains.kI().getAsDouble(),
                  V2_DeltaConstants.DRIVE_CONSTANTS.autoRotationGains.kD().getAsDouble())),
          RobotConfig.fromGUISettings(),
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          drive);
      OP_BUCKS_1 = PathPlannerPath.fromPathFile("LEFT_OP_BUCKS_1");
      PathPlannerPath OP_2 = PathPlannerPath.fromPathFile("LEFT_OP_2");
      return Commands.sequence(
          Commands.runOnce(
              () ->
                  V2_DeltaRobotState.resetPose(
                      AllianceFlipUtil.apply(OP_BUCKS_1.getStartingHolonomicPose().get()))),
          Commands.waitSeconds(1.5),
          intake
              .deploy()
              .alongWith(intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)),
          AutoBuilder.followPath(OP_BUCKS_1),
          drive.runOnce(drive::stop),
          V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper).withTimeout(5.0),
          AutoBuilder.followPath(OP_2)
              .alongWith(intake.deploy(), V2_DeltaCompositeCommands.hold(clopper, shooter)),
          Commands.runOnce(drive::stop),
          V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper));
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }
  }
}
