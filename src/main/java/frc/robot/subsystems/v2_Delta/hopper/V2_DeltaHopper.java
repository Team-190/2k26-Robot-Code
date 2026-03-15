package frc.robot.subsystems.v2_Delta.hopper;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import org.littletonrobotics.junction.Logger;

public class V2_DeltaHopper extends SubsystemBase {
  private final GenericRoller rollerFloor;
  private final GenericRoller ballTunnel;

  public V2_DeltaHopper(GenericRollerIO rollerFloorIO, GenericRollerIO ballTunnelIO) {
    setName("Hopper");

    rollerFloor =
        new GenericRoller(
            rollerFloorIO, this, V2_DeltaHopperConstants.ROLLERFLOOR_CONSTANTS, "rollerFloor");
    ballTunnel =
        new GenericRoller(
            ballTunnelIO, this, V2_DeltaHopperConstants.BALLTUNNEL_CONSTANTS, "ballTunnel");
  }

  @Override
  public void periodic() {
    rollerFloor.periodic();
    ballTunnel.periodic();

    Logger.recordOutput(
        "Hopper/BallTunnel/Voltage", ballTunnel.getVoltageGoalVolts().getSetpoint());
    Logger.recordOutput(
        "Hopper/RollerFloor/Voltage", rollerFloor.getVoltageGoalVolts().getSetpoint());
  }

  public Command setRollerFloorVoltage(Voltage voltage) {
    return rollerFloor.setVoltage(voltage);
  }

  public Command feedShooterRollerFloor() {
    return setRollerFloorVoltage(V2_DeltaHopperConstants.ROLLER_FLOOR_FEED_VOLTAGE);
  }

  public Command outtakeRollerFloor() {
    return setRollerFloorVoltage(V2_DeltaHopperConstants.OUTTAKE_VOLTAGE);
  }

  public Command stopRollerFloor() {
    return setRollerFloorVoltage(Volts.of(0.0));
  }

  public Command setBallTunnelVoltage(Voltage voltage) {
    return ballTunnel.setVoltage(voltage);
  }

  public Command feedShooterBallTunnel() {
    return setBallTunnelVoltage(V2_DeltaHopperConstants.ROLLER_FLOOR_FEED_VOLTAGE);
  }

  public Command outtakeBallTunnel() {
    return setBallTunnelVoltage(V2_DeltaHopperConstants.OUTTAKE_VOLTAGE);
  }

  public Command stopBallTunnel() {
    return setBallTunnelVoltage(Volts.of(0.0));
  }

  public Command intake() {
    return Commands.parallel(feedShooterRollerFloor(), feedShooterBallTunnel());
  }

  public Command outtake() {
    return Commands.parallel(outtakeRollerFloor(), outtakeBallTunnel());
  }
}
