package frc.robot.commands.v1_Gamma.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.subsystems.shared.turret.Turret;
import frc.robot.subsystems.v1_Gamma.swank.V1_GammaSwank;

public class V1_GammaAutonomousTest {

  public static void loadAutoTrajectories(SwerveDrive drive) {
    drive.getAutoFactory().cache().loadTrajectory("Trajectory Name");
  }

  public static final Command spindexerTest(V1_GammaSwank clanker) {
    return clanker.setVoltage(3.0);
  }

  public static final Command swankTest(V1_GammaSwank swank) {
    return swank.setVoltage(3.0);
  }

  public static final Command turretTest(Turret turret) {
    return turret.setTurretVoltage(3.0);
  }
}
