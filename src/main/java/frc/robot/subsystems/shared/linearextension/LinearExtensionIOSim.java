package frc.robot.subsystems.shared.linearextension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.LinearConstraints;

public class LinearExtensionIOSim implements LinearExtensionIO {

  private final DCMotorSim motorSim;

  private final ProfiledPIDController feedback;

  private final SimpleMotorFeedforward feedforward;

  private double appliedVolts = 0.0;

  public LinearExtensionIOSim(LinearExtensionConstants constants) {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                constants.MOTOR_CONFIG, constants.MOMENT_OF_INERTIA, constants.GEAR_RATIO),
            constants.MOTOR_CONFIG);

    feedback =
        new ProfiledPIDController(
            constants.GAINS.kP().get(),
            0.0,
            constants.GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                constants.CONSTRAINTS.maxVelocity().get().in(MetersPerSecond),
                constants.CONSTRAINTS.maxAcceleration().get().in(MetersPerSecondPerSecond)));

    feedback.setTolerance(constants.CONSTRAINTS.goalTolerance().get().in(Meters));
    feedforward =
        new SimpleMotorFeedforward(constants.GAINS.kS().get(), constants.GAINS.kV().get());
  }

  @Override
  public void updateInputs(LinearExtensionIOInputs inputs) {
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    motorSim.setInputVoltage(appliedVolts);

    motorSim.update(GompeiLib.getLoopPeriod());

    inputs.position = Distance.ofBaseUnits(motorSim.getAngularPositionRad(), Meters);
    inputs.velocity = MetersPerSecond.of(motorSim.getAngularVelocityRadPerSec());
    inputs.supplyCurrent = Amps.of(motorSim.getCurrentDrawAmps());
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.positionGoal = Distance.ofBaseUnits(feedback.getGoal().position, Meters);
    inputs.positionSetpoint = Distance.ofBaseUnits(feedback.getGoal().position, Meters);
    inputs.positionError = Distance.ofBaseUnits(feedback.getPositionError(), Meters);
  }

  @Override
  /** Set voltage. */
  public void setVoltage(Voltage volts) {
    appliedVolts = volts.baseUnitMagnitude();
  }

  @Override
  /** Set closed loop position setpoint. */
  public void setPositionGoal(Distance position) {
    appliedVolts =
        feedback.calculate(motorSim.getAngularPositionRad(), position.baseUnitMagnitude())
            + feedforward.calculate(feedback.getSetpoint().velocity);
  }

  @Override
  public void setPosition(Distance position) {
    motorSim.setState(position.baseUnitMagnitude(), motorSim.getAngularVelocityRadPerSec());
    feedback.reset(position.baseUnitMagnitude(), motorSim.getAngularVelocityRadPerSec());
  }

  @Override
  public void setPID(Gains gains) {
    feedback.setPID(gains.kP().getAsDouble(), gains.kI().getAsDouble(), gains.kD().getAsDouble());
  }

  @Override
  public void setProfile(LinearConstraints constraints) {
    feedback.setConstraints(
        new Constraints(
            constraints.maxVelocity().getRawValue(), constraints.maxAcceleration().getRawValue()));
    feedback.setTolerance(constraints.goalTolerance().getRawValue());
  }

  @Override
  /** Check if the linkage is within tolerance */
  public boolean atGoal() {
    return feedback.atGoal();
  }
}
