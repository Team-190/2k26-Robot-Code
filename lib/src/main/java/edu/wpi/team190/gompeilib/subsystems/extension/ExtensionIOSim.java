package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import java.util.Arrays;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.system.Models;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.units.measure.*;

public class ExtensionIOSim implements ExtensionIO {
  private final ElevatorSim sim;

  private Voltage appliedVolts;
  private boolean isClosedLoop;
  private GainSlot gainSlot;

  private final ProfiledPIDController feedback;
  private ElevatorFeedforward feedforward;

  private final ExtensionConstants constants;

  public ExtensionIOSim(ExtensionConstants constants) {
    sim =
        new ElevatorSim(
            Models.elevatorFromPhysicalConstants(
                constants.extensionParameters.EXTENSION_MOTOR_CONFIG(),
                constants.extensionParameters.CARRIAGE_MASS_KG(),
                constants.drumRadius,
                constants.extensionGearRatio),
            constants.extensionParameters.EXTENSION_MOTOR_CONFIG(),
            constants.extensionParameters.MIN_LENGTH().in(Meters),
            constants.extensionParameters.MAX_LENGTH().in(Meters),
            constants.verticalGravity,
            constants.extensionParameters.MIN_LENGTH().in(Meters));

    appliedVolts = Volts.of(0.0);
    isClosedLoop = true;
    gainSlot = GainSlot.ZERO;

    feedback =
        new ProfiledPIDController(
            constants.slot0Gains.kP().get(),
            0,
            constants.slot0Gains.kD().get(),
            new TrapezoidProfile.Constraints(
                constants.constraints.maxVelocity().get().in(MetersPerSecond),
                constants.constraints.maxAcceleration().get().in(MetersPerSecondPerSecond)));

    feedforward =
        new ElevatorFeedforward(
            constants.slot0Gains.kS().get(),
            constants.slot0Gains.kG().get(),
            constants.slot0Gains.kV().get(),
            constants.slot0Gains.kA().get());

    this.constants = constants;
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    if (isClosedLoop) {
      appliedVolts =
          Volts.of(
              feedback.calculate(sim.getPosition())
                  + feedforward.calculate((feedback.getSetpoint().velocity)));
    }

    appliedVolts = Volts.of(Math.clamp(appliedVolts.in(Volts), -12, 12));

    sim.setInputVoltage(appliedVolts.in(Volts));
    sim.update(GompeiLib.getLoopPeriod());

    inputs.position = Meters.of(sim.getPosition());
    inputs.velocity = MetersPerSecond.of(sim.getVelocity());
    inputs.linearVelocitySetpoint = MetersPerSecond.of(feedback.getSetpoint().velocity);
    inputs.acceleration =
        MetersPerSecondPerSecond.of(-1.0); // TODO: Replace with calculation based on velocity

    inputs.appliedVolts = new double[constants.extensionParameters.NUM_MOTORS()];
    inputs.supplyCurrentAmps = new double[constants.extensionParameters.NUM_MOTORS()];
    inputs.torqueCurrentAmps = new double[constants.extensionParameters.NUM_MOTORS()];
    inputs.temperatureCelsius = new double[constants.extensionParameters.NUM_MOTORS()];

    Arrays.fill(inputs.appliedVolts, appliedVolts.in(Volts));
    Arrays.fill(inputs.supplyCurrentAmps, sim.getCurrentDraw());
    Arrays.fill(inputs.torqueCurrentAmps, sim.getCurrentDraw());
    Arrays.fill(inputs.temperatureCelsius, 0.0);

    inputs.positionGoalMeters = Meters.of(feedback.getGoal().position);
    inputs.positionSetpointMeters = Meters.of(feedback.getSetpoint().position);
    inputs.positionErrorMeters = Meters.of(feedback.getPositionError());

    inputs.gainSlot = gainSlot;
  }

  @Override
  public void setVoltageGoal(Voltage volts) {
    isClosedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void setPositionGoal(Distance position) {
    if (!isClosedLoop) {
      feedback.reset(sim.getPosition(), sim.getVelocity());
    }
    isClosedLoop = true;
    feedback.setGoal(position.in(Meters));
  }

  @Override
  public boolean atVoltageGoal(Voltage voltageReference) {
    return appliedVolts.isNear(voltageReference, Millivolt.of(500));
  }

  @Override
  public boolean atPositionGoal(Distance positionReference) {
    return Meters.of(sim.getPosition())
        .isNear(positionReference, constants.constraints.goalTolerance().get());
  }

  @Override
  public void setPosition(Distance position) {
    sim.setState(position.in(Meters), sim.getVelocity());
  }

  @Override
  public void setGainSlot(GainSlot gainSlot) {
    this.gainSlot = gainSlot;
    switch (gainSlot) {
      case ZERO:
        feedback.setPID(constants.slot0Gains.kP().get(), 0.0, constants.slot0Gains.kD().get());
        break;
      case ONE:
        feedback.setPID(constants.slot1Gains.kP().get(), 0.0, constants.slot1Gains.kD().get());
        break;
      case TWO:
        feedback.setPID(constants.slot2Gains.kP().get(), 0.0, constants.slot2Gains.kD().get());
        break;
      default:
        throw new IllegalStateException("Unknown gain slot: " + gainSlot);
    }
  }

  @Override
  public void updateGains(Gains gains, GainSlot gainSlot) {
    feedback.setPID(gains.kP().get(), gains.kI().get(), gains.kD().get());
    feedforward =
        new ElevatorFeedforward(
            gains.kS().get(), gains.kG().get(), gains.kV().get(), gains.kA().get());
  }

  @Override
  public void updateConstraints(LinearConstraints constraints) {
    feedback.setConstraints(
        new TrapezoidProfile.Constraints(
            constraints.maxVelocity().get().in(MetersPerSecond),
            constraints.maxAcceleration().get().in(MetersPerSecondPerSecond)));
    feedback.setTolerance(constraints.goalTolerance().get().in(Meters));
  }
}
