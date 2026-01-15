package frc.robot.subsystems.v0_Funky.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import frc.robot.subsystems.v0_Funky.hood.V0_FunkyHoodConstants.HoodGoal;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V0_FunkyHood extends SubsystemBase {
  private final V0_FunkyHoodIO io;
  private final V0_FunkyHoodIOInputsAutoLogged inputs;

  private SysIdRoutine characterizationRoutine;

  private boolean isClosedLoop;
  private HoodGoal goal;

  public V0_FunkyHood(V0_FunkyHoodIO io) {
    inputs = new V0_FunkyHoodIOInputsAutoLogged();
    this.io = io;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Arm/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));

    isClosedLoop = false;
    goal = HoodGoal.STOW;
  }

  public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        if (isClosedLoop) {
          io.setPosition(goal.getAngle());
        }
  }

  public Command setGoal(HoodGoal goal) {
    return Commands.run(() -> {this.goal = goal;
    isClosedLoop = true;});
  }

  @AutoLogOutput(key = "Hood/At Goal")
  public boolean atGoal() {
    return io.atGoal();
  }

  public Command waitUntilHoodAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod()).andThen(Commands.waitUntil(this::atGoal));
  }

  public void setPID(double kp, double kd) {
    io.setPID(kp, 0.0, kd);
  }

  public void setFeedforward(double ks, double kv, double ka) {
    io.setFeedforward(ks, kv, ka);
  }

  public void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    io.setProfile(
        maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared, goalToleranceRadians);
  }

  public Command runSysId() {
    isClosedLoop = false;
    return Commands.sequence(
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kReverse));
  }
}