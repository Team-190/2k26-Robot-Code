package frc.robot.util;

import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.TunableUpdaterRegistry;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.v2_Turnover.V2_TurnoverConstants;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverShooter;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverShooterConstants;

public class LTNUpdater {
  public static void registerV2(SwerveDrive drive, Intake intake, V2_TurnoverShooter shooter) {
    TunableUpdaterRegistry.registerGains(
        V2_TurnoverConstants.DRIVE_GAINS,
        g -> {
          drive.setPIDGains(
              g.getKP(),
              g.getKD(),
              V2_TurnoverConstants.TURN_GAINS.getKP(),
              V2_TurnoverConstants.TURN_GAINS.getKD());
          drive.setFFGains(g.getKS(), g.getKV());
        });
    TunableUpdaterRegistry.registerGains(
        V2_TurnoverConstants.TURN_GAINS,
        g ->
            drive.setPIDGains(
                V2_TurnoverConstants.DRIVE_GAINS.getKP(),
                V2_TurnoverConstants.DRIVE_GAINS.getKD(),
                g.getKP(),
                g.getKD()));

    TunableUpdaterRegistry.registerGains(
        IntakeConstants.LINKAGE_CONSTANTS.gains, intake.getLinkage()::setGains);

    TunableUpdaterRegistry.registerGains(
        V2_TurnoverShooterConstants.HOOD_CONSTANTS.gains, shooter::setHoodGains);
    TunableUpdaterRegistry.registerConstraints(
        V2_TurnoverShooterConstants.HOOD_CONSTANTS.constraints,
        c -> shooter.setHoodConstraints((AngularPositionConstraints) c));

    TunableUpdaterRegistry.registerConstraints(
        V2_TurnoverShooterConstants.SHOOT_CONSTANTS.constraints,
        c -> shooter.setFlywheelConstraints((AngularVelocityConstraints) c));
    TunableUpdaterRegistry.registerGains(
        V2_TurnoverShooterConstants.SHOOT_CONSTANTS.voltageGains, shooter::setFlywheelGains);

    TunableUpdaterRegistry.registerConstraints(
        V2_TurnoverShooterConstants.TURRET_CONSTANTS.constraints,
        c -> shooter.setTurretConstraints((AngularPositionConstraints) c));
    TunableUpdaterRegistry.registerGains(
        V2_TurnoverShooterConstants.TURRET_CONSTANTS.gains, shooter::setTurretGains);
  }
}
