package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.subsystems.v0_Funky.shooter.V0_FunkyShooter;

public class AutonomousCommands {

  public static void loadAutoTrajectories(SwerveDrive drive) {
    drive.getAutoFactory().cache().loadTrajectory("Trajectory Name");
  }

  public static Command moveTurret(V0_FunkyShooter shooter, double angleDegrees) {
    // return Commands.runOnce(() -> shooter.setTurretGoal(Rotation2d.fromDegrees(angleDegrees)));

    return shooter.setVoltageCommand(5);
  }
}
