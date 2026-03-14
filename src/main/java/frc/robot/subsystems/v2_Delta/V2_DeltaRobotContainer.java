package frc.robot.subsystems.v2_Delta;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.*;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFXSim;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.subsystems.v2_Delta.hopper.V2_DeltaHopper;
import frc.robot.subsystems.v2_Delta.hopper.V2_DeltaHopperConstants;
import org.littletonrobotics.junction.Logger;

public class V2_DeltaRobotContainer implements RobotContainer {
  private V2_DeltaHopper hopper;

  public V2_DeltaRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V2_DELTA:
          hopper =
              new V2_DeltaHopper(
                  new GenericRollerIOTalonFX(V2_DeltaHopperConstants.ROLLERFLOOR_CONSTANTS),
                  new GenericRollerIOTalonFX(V2_DeltaHopperConstants.BALLTUNNEL_CONSTANTS));
          break;
        case V2_DELTA_SIM:
          hopper =
              new V2_DeltaHopper(
                  new GenericRollerIOTalonFXSim(V2_DeltaHopperConstants.ROLLERFLOOR_CONSTANTS),
                  new GenericRollerIOTalonFXSim(V2_DeltaHopperConstants.BALLTUNNEL_CONSTANTS));
          break;
        default:
          if (hopper == null) {
            hopper = new V2_DeltaHopper(new GenericRollerIO() {}, new GenericRollerIO() {});
          }
      }
    }
  }

  private void configureButtonBindings() {
    //
  }

  private void configureAutos() {
    //
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("Hopper/Alive", true);
  }

  @Override
  public Command getAutonomousCommand() {
    return hopper.feedShooterBallTunnel().andThen(hopper.feedShooterRollerFloor());
  }
}
