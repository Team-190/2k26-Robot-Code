package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.LinearProfile;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import java.util.Arrays;

public class GenericFlywheelIOSim implements GenericFlywheelIO {

  private final FlywheelSim motorSim;

  private Voltage appliedVolts;
  private boolean isClosedLoop;
  private GainSlot gainSlot;

  private final PIDController feedback;
  private final SimpleMotorFeedforward feedforward;
  private final LinearProfile profile;

  GenericFlywheelConstants constants;

  private Angle accumulatedPosition;

  public GenericFlywheelIOSim(GenericFlywheelConstants constants) {
    motorSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                constants.motorConfig, constants.momentOfInertia, constants.gearRatio),
            constants.motorConfig);

    appliedVolts = Volts.of(0.0);
    isClosedLoop = false;

    feedback =
        new PIDController(
            constants.voltageGains.kP().get(), 0.0, constants.voltageGains.kD().get());
    feedback.setTolerance(constants.constraints.goalTolerance().get().in(RadiansPerSecond));
    feedforward =
        new SimpleMotorFeedforward(
            constants.voltageGains.kS().get(), constants.voltageGains.kV().get());
    profile =
        new LinearProfile(
            constants.constraints.maxAcceleration().get().in(RadiansPerSecondPerSecond),
            constants.constraints.maxVelocity().get().in(RadiansPerSecond),
            1 / GompeiLib.getLoopPeriod());

    this.constants = constants;

    accumulatedPosition = Radians.of(0.0);
  }

  @Override
  public void updateInputs(GenericFlywheelIOInputs inputs) {
    if (isClosedLoop)
      appliedVolts =
          Volts.of(
              feedback.calculate(motorSim.getAngularVelocityRadPerSec())
                  + feedforward.calculate(feedback.getSetpoint()));

    appliedVolts = Volts.of(MathUtil.clamp(appliedVolts.in(Volts), -12.0, 12.0));
    motorSim.setInputVoltage(appliedVolts.in(Volts));
    motorSim.update(1.0 / GompeiLib.getLoopPeriod());

    accumulatedPosition =
        accumulatedPosition.plus(
            motorSim.getAngularVelocity().times(Seconds.of(GompeiLib.getLoopPeriod())));

    inputs.position = Rotation2d.fromRadians(accumulatedPosition.in(Radians));
    inputs.velocity = motorSim.getAngularVelocity();

    Arrays.fill(inputs.appliedVolts, appliedVolts.in(Volts));
    Arrays.fill(inputs.supplyCurrentAmps, motorSim.getCurrentDrawAmps());
    Arrays.fill(inputs.torqueCurrentAmps, motorSim.getCurrentDrawAmps());

    inputs.velocityGoal = RadiansPerSecond.of(profile.getGoal());
    inputs.velocitySetpoint = RadiansPerSecond.of(feedback.getSetpoint());
    inputs.velocityError = RadiansPerSecond.of(feedback.getError());

    inputs.gainSlot = gainSlot;
  }

  @Override
  public void setVoltageGoal(Voltage voltageGoal) {
    isClosedLoop = false;
    appliedVolts = voltageGoal;
  }

  @Override
  public void setVelocityGoal(AngularVelocity velocityGoal) {
    isClosedLoop = true;
    profile.setGoal(velocityGoal.in(RadiansPerSecond), motorSim.getAngularVelocityRadPerSec());
    appliedVolts =
        Volts.of(
            feedback.calculate(motorSim.getAngularVelocityRadPerSec(), profile.calculateSetpoint())
                + feedforward.calculate(feedback.getSetpoint()));
  }

  @Override
  public void setNeutralControl() {
    appliedVolts = Volts.of(0.0);
    profile.reset();
  }

  @Override
  public boolean atVoltageGoal(Voltage voltageReference) {
    return appliedVolts.isNear(voltageReference, Millivolts.of(500));
  }

  @Override
  public boolean atVelocityGoal(AngularVelocity velocityReference) {
    return motorSim
        .getAngularVelocity()
        .isNear(velocityReference, constants.constraints.goalTolerance().get(RadiansPerSecond));
  }

  @Override
  public void updateGains(Gains gains, GainSlot gainSlot) {
    this.gainSlot = gainSlot;
    feedback.setPID(gains.kP().get(), gains.kI().get(), gains.kD().get());
    feedforward.setKs(gains.kS().get());
    feedforward.setKv(gains.kV().get());
    feedforward.setKa(gains.kA().get());
  }

  @Override
  public void updateConstraints(AngularVelocityConstraints constraints) {
    profile.setMaxAcceleration(constraints.maxAcceleration().get().in(RadiansPerSecondPerSecond));
    profile.setMaxVelocity(constraints.maxVelocity().get().in(RadiansPerSecond));
    feedback.setTolerance(constraints.goalTolerance().get().in(RadiansPerSecond));
  }
}
