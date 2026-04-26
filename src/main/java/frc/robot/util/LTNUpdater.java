package frc.robot.util;

import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.TunableUpdaterRegistry;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.v2_TurnOver.V2_TurnOverConstants;
import frc.robot.subsystems.v2_TurnOver.shooter.V2_TurnOverShooter;
import frc.robot.subsystems.v2_TurnOver.shooter.V2_TurnOverShooterConstants;

public class LTNUpdater {
  public static void registerV2(SwerveDrive drive, Intake intake, V2_TurnOverShooter shooter) {
    TunableUpdaterRegistry.registerGains(
        V2_TurnOverConstants.DRIVE_GAINS,
        g -> {
          drive.setPIDGains(
              g.getKP(),
              g.getKD(),
              V2_TurnOverConstants.TURN_GAINS.getKP(),
              V2_TurnOverConstants.TURN_GAINS.getKD());
          drive.setFFGains(g.getKS(), g.getKV());
        });
    TunableUpdaterRegistry.registerGains(
        V2_TurnOverConstants.TURN_GAINS,
        g ->
            drive.setPIDGains(
                V2_TurnOverConstants.DRIVE_GAINS.getKP(),
                V2_TurnOverConstants.DRIVE_GAINS.getKD(),
                g.getKP(),
                g.getKD()));

    TunableUpdaterRegistry.registerGains(
        IntakeConstants.LINKAGE_CONSTANTS.gains, intake.getLinkage()::setGains);

    TunableUpdaterRegistry.registerGains(
        V2_TurnOverShooterConstants.HOOD_CONSTANTS.gains, shooter::setHoodGains);
    TunableUpdaterRegistry.registerConstraints(
        V2_TurnOverShooterConstants.HOOD_CONSTANTS.constraints,
        c -> shooter.setHoodConstraints((AngularPositionConstraints) c));

    TunableUpdaterRegistry.registerConstraints(
        V2_TurnOverShooterConstants.SHOOT_CONSTANTS.constraints,
        c -> shooter.setFlywheelConstraints((AngularVelocityConstraints) c));
    TunableUpdaterRegistry.registerGains(
        V2_TurnOverShooterConstants.SHOOT_CONSTANTS.voltageGains, shooter::setFlywheelGains);

    TunableUpdaterRegistry.registerConstraints(
        V2_TurnOverShooterConstants.TURRET_CONSTANTS.constraints,
        c -> shooter.setTurretConstraints((AngularPositionConstraints) c));
    TunableUpdaterRegistry.registerGains(
        V2_TurnOverShooterConstants.TURRET_CONSTANTS.gains, shooter::setTurretGains);
  }
}
