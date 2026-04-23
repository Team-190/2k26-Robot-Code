package frc.robot.util;

import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.TunableUpdaterRegistry;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.v2_Delta.V2_DeltaConstants;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooter;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooterConstants;

public class LTNUpdater {
  public static void registerV2(SwerveDrive drive, Intake intake, V2_DeltaShooter shooter) {
    TunableUpdaterRegistry.registerGains(
        V2_DeltaConstants.DRIVE_GAINS,
        g -> {
          drive.setPIDGains(
              g.getKP(),
              g.getKD(),
              V2_DeltaConstants.TURN_GAINS.getKP(),
              V2_DeltaConstants.TURN_GAINS.getKD());
          drive.setFFGains(g.getKS(), g.getKV());
        });
    TunableUpdaterRegistry.registerGains(
        V2_DeltaConstants.TURN_GAINS,
        g ->
            drive.setPIDGains(
                V2_DeltaConstants.DRIVE_GAINS.getKP(),
                V2_DeltaConstants.DRIVE_GAINS.getKD(),
                g.getKP(),
                g.getKD()));

    TunableUpdaterRegistry.registerGains(
        IntakeConstants.LINKAGE_CONSTANTS.gains, intake.getLinkage()::setGains);

    TunableUpdaterRegistry.registerGains(
        V2_DeltaShooterConstants.HOOD_CONSTANTS.gains, shooter::setHoodGains);
    TunableUpdaterRegistry.registerConstraints(
        V2_DeltaShooterConstants.HOOD_CONSTANTS.constraints,
        c -> shooter.setHoodConstraints((AngularPositionConstraints) c));

    TunableUpdaterRegistry.registerConstraints(
        V2_DeltaShooterConstants.SHOOT_CONSTANTS.constraints,
        c -> shooter.setFlywheelConstraints((AngularVelocityConstraints) c));
    TunableUpdaterRegistry.registerGains(
        V2_DeltaShooterConstants.SHOOT_CONSTANTS.voltageGains, shooter::setFlywheelGains);

    TunableUpdaterRegistry.registerConstraints(
        V2_DeltaShooterConstants.TURRET_CONSTANTS.constraints,
        c -> shooter.setTurretConstraints((AngularPositionConstraints) c));
    TunableUpdaterRegistry.registerGains(
        V2_DeltaShooterConstants.TURRET_CONSTANTS.gains, shooter::setTurretGains);
  }
}
