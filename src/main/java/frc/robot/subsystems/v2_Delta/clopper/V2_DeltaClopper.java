package frc.robot.subsystems.v2_Delta.clopper;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class V2_DeltaClopper extends SubsystemBase {
  private final GenericRoller rollerFloor;
  private final GenericRoller ballTunnel;
  private final DigitalInput beamBreak;

  @Getter private final Trigger beamBreakTrigger;

  public V2_DeltaClopper(GenericRollerIO rollerFloorIO, GenericRollerIO ballTunnelIO) {
    setName("Hopper");

    rollerFloor =
        new GenericRoller(
            rollerFloorIO, this, V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS, " Floor");
    ballTunnel =
        new GenericRoller(
            ballTunnelIO, this, V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS, " Tunnel");

    beamBreak = new DigitalInput(V2_DeltaClopperConstants.BEAM_BREAK_ID);
    beamBreakTrigger = new Trigger(beamBreak::get);
  }

  @Override
  public void periodic() {
    rollerFloor.periodic();
    ballTunnel.periodic();
    Logger.recordOutput(getName() + "/Beam Break", beamBreak.get());
  }

  public Command setRollerFloorVoltage(Voltage voltage) {
    return Commands.runOnce(() -> rollerFloor.setVoltageGoal(voltage));
  }

  public Command feedShooterRollerFloor() {
    return setRollerFloorVoltage(V2_DeltaClopperConstants.ROLLER_FLOOR_FEED_VOLTAGE);
  }


  public Command stopRollerFloor() {
    return setRollerFloorVoltage(Volts.of(0.0));
  }

  public Command setBallTunnelVoltage(Voltage voltage) {
    return Commands.runOnce(() -> ballTunnel.setVoltageGoal(voltage));
  }

  public Command feedShooterBallTunnel() {
    return setBallTunnelVoltage(V2_DeltaClopperConstants.ROLLER_FLOOR_FEED_VOLTAGE);
  }

  public Command outtakeBallTunnel() {
    return setBallTunnelVoltage(V2_DeltaClopperConstants.OUTTAKE_VOLTAGE);
  }

  public Command stopBallTunnel() {
    return setBallTunnelVoltage(Volts.of(0.0));
  }

  public Command intake() {
    return Commands.parallel(feedShooterRollerFloor(), feedShooterBallTunnel());
  }

  public Command outtake() {
    return outtakeBallTunnel();
  }

  public Command deployClimber() {
    return Commands.sequence(
        Commands.run(() -> rollerFloor.setVoltageGoal(Volts.of(-12)))
            .until(beamBreakTrigger.negate()),
        Commands.run(() -> rollerFloor.setVoltageGoal(Volts.of(-12))).until(beamBreakTrigger),
        Commands.run(() -> rollerFloor.setVoltageGoal(Volts.of(-6)))
            .until(beamBreakTrigger.negate()),
        Commands.run(() -> rollerFloor.setVoltageGoal(Volts.of(-6))).until(beamBreakTrigger),
        Commands.runOnce(() -> rollerFloor.setVoltageGoal(Volts.of(0.0))));
  }

  public Command climbUp() {
    return Commands.sequence(
        Commands.run(() -> rollerFloor.setVoltageGoal(Volts.of(-12))).until(beamBreakTrigger),
        Commands.run(() -> rollerFloor.setVoltageGoal(Volts.of(-12)))
            .until(beamBreakTrigger.negate()),
        Commands.run(() -> rollerFloor.setVoltageGoal(Volts.of(-6))).until(beamBreakTrigger),
        Commands.run(() -> rollerFloor.setVoltageGoal(Volts.of(0.0))));
  }

  public Command climbDown() {
    return Commands.sequence(
        Commands.run(() -> rollerFloor.setVoltageGoal(Volts.of(-6)))
            .until(beamBreakTrigger.negate()),
        Commands.run(() -> rollerFloor.setVoltageGoal(Volts.of(-6))).until(beamBreakTrigger),
        Commands.run(() -> rollerFloor.setVoltageGoal(Volts.of(-3)))
            .until(beamBreakTrigger.negate()),
        Commands.run(() -> rollerFloor.setVoltageGoal(Volts.of(0))));
  }
}
