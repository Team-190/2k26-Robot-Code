package first.robot.shared.util;

import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.shared.subsystems.intake.Intake;
import first.robot.shared.subsystems.intake.IntakeConstants;
import first.robot.v2_Turnover.V2_TurnoverConstants;
import first.robot.v2_Turnover.subsystems.shooter.V2_TurnoverShooter;
import first.robot.v2_Turnover.subsystems.shooter.V2_TurnoverShooterConstants;

public class LTNUpdater {
  public static void registerV2(SwerveDrive drive, Intake intake, V2_TurnoverShooter shooter) {
    V2_TurnoverConstants.DRIVE_GAINS.onChange(
        g -> {
          drive.setPIDGains(
              g.getKP(),
              g.getKD(),
              V2_TurnoverConstants.TURN_GAINS.getKP(),
              V2_TurnoverConstants.TURN_GAINS.getKD());
          drive.setFFGains(g.getKS(), g.getKV());
        });
    V2_TurnoverConstants.TURN_GAINS.onChange(
        g ->
            drive.setPIDGains(
                V2_TurnoverConstants.DRIVE_GAINS.getKP(),
                V2_TurnoverConstants.DRIVE_GAINS.getKD(),
                g.getKP(),
                g.getKD()));

    IntakeConstants.LINKAGE_CONSTANTS.gains.onChange(intake.getLinkage()::setGains);

    V2_TurnoverShooterConstants.HOOD_CONSTANTS.gains.onChange(shooter::setHoodGains);
    V2_TurnoverShooterConstants.HOOD_CONSTANTS.constraints.onChange(shooter::setHoodConstraints);

    V2_TurnoverShooterConstants.SHOOT_CONSTANTS.constraints.onChange(
        shooter::setFlywheelConstraints);
    V2_TurnoverShooterConstants.SHOOT_CONSTANTS.voltageGains.onChange(shooter::setFlywheelGains);

    V2_TurnoverShooterConstants.TURRET_CONSTANTS.constraints.onChange(
        shooter::setTurretConstraints);
    V2_TurnoverShooterConstants.TURRET_CONSTANTS.gains.onChange(shooter::setTurretGains);
  }
}
