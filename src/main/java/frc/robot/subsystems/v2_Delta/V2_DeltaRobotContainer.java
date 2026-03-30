package frc.robot.subsystems.v2_Delta;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFXSim;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopper;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopperConstants;

public class V2_DeltaRobotContainer implements RobotContainer {
  private V2_DeltaClopper hopper;

  public V2_DeltaRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V2_DELTA:
          hopper =
              new V2_DeltaClopper(
                  new GenericRollerIOTalonFX(V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS),
                  new GenericRollerIOTalonFX(V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS));
          break;
        case V2_DELTA_SIM:
          hopper =
              new V2_DeltaClopper(
                  new GenericRollerIOTalonFXSim(V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS),
                  new GenericRollerIOTalonFXSim(V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS));
          break;
        default:
      }
    }
    if (hopper == null) {
      hopper = new V2_DeltaClopper(new GenericRollerIO() {}, new GenericRollerIO() {});
    }
  }

  private void configureButtonBindings() {
    //
  }

  private void configureAutos() {
    //
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public Command getAutonomousCommand() {
    return hopper.feedShooterBallTunnel().andThen(hopper.feedShooterRollerFloor());
  }
}
