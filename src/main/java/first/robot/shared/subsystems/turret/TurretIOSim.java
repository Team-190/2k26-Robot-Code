package first.robot.shared.subsystems.turret;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import java.util.Random;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.Models;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

public class TurretIOSim implements TurretIO {

  private final TurretConstants constants;

  private final ProfiledPIDController feedback;
  private final SimpleMotorFeedforward feedforward;

  private final Random random = new Random();

  private final DCMotorSim sim;

  private Rotation2d positionGoal = new Rotation2d();
  private double appliedVolts = 0.0;

  private Angle encoderValue1;
  private Angle encoderValue2;
  private Angle realAngle;

  public TurretIOSim(TurretConstants constants) {
    this.constants = constants;
    sim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                constants.motorConfig, constants.momentOfInertia, constants.gearRatio),
            constants.motorConfig);

    feedback =
        new ProfiledPIDController(
            constants.gains.kP().get(),
            0.0,
            constants.gains.kD().get(),
            new TrapezoidProfile.Constraints(
                constants.constraints.maxVelocity().get().in(RadiansPerSecond),
                constants.constraints.maxAcceleration().get().in(RadiansPerSecondPerSecond)));
    feedback.setTolerance(constants.constraints.goalTolerance().get().in(Radians));

    feedback.disableContinuousInput();

    feedforward =
        new SimpleMotorFeedforward(constants.gains.kS().get(), constants.gains.kV().get());
    realAngle = Radian.of(MathUtil.angleModulus((Timer.getTimestamp() * 0.1)));

    encoderValue1 = Angle.ofBaseUnits(0, Radian);
    encoderValue2 = Angle.ofBaseUnits(0, Radian);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    appliedVolts = Math.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
    sim.update(GompeiLib.getLoopPeriod());

    realAngle = Radian.of(sim.getAngularPosition());

    encoderValue1 =
        realAngle
            .times(
                constants.turretAngleCalculation.gear0ToothCount()
                    / constants.turretAngleCalculation.gear1ToothCount())
            .plus(
                // Add some noise
                Degree.of(random.nextDouble() * 0.04 - 0.5));
    encoderValue2 =
        realAngle
            .times(
                constants.turretAngleCalculation.gear0ToothCount()
                    / constants.turretAngleCalculation.gear2ToothCount())
            .plus(
                // Add some noise
                Degree.of(random.nextDouble() * 0.04 - 0.5));

    inputs.angle = Rotation2d.fromRadians(realAngle.in(Radian));
    inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocity());
    inputs.appliedVoltage = Volts.of(appliedVolts);
    inputs.supplyCurrent = Amp.of(sim.getCurrentDraw());
    inputs.positionGoal = positionGoal;
    inputs.positionSetpoint = Rotation2d.fromRadians(feedback.getSetpoint().position);
    inputs.positionError = Rotation2d.fromRadians(feedback.getPositionError());

    inputs.encoder1Position = new Rotation2d(encoderValue1);
    inputs.encoder2Position = new Rotation2d(encoderValue2);
  }

  @Override
  public void setVoltageGoal(Voltage volts) {
    appliedVolts = volts.in(Volt);
  }

  @Override
  public void setPositionGoal(
      Rotation2d goal, AngularVelocity velocity, double feedforwardVoltage) {
    appliedVolts =
        feedback.calculate(sim.getAngularPosition(), goal.getRadians())
            + feedforward.calculate(feedback.getSetpoint().velocity + velocity.baseUnitMagnitude());

    appliedVolts += Math.copySign(feedforwardVoltage, appliedVolts);
    positionGoal = goal;
  }

  public void setPositionGoal(Rotation2d goal, double feedforwardVoltage) {
    setPositionGoal(goal, RadiansPerSecond.zero(), feedforwardVoltage);
  }

  @Override
  public boolean atPositionGoal(Rotation2d positionReference) {
    return sim.getAngularPosition() - positionReference.getRadians()
        <= constants.constraints.goalTolerance().get().in(Radians);
  }

  @Override
  public void setPosition(Rotation2d angle) {
    sim.setState(angle.getRadians(), 0.0);
    feedback.reset(angle.getRadians(), 0.0);
  }

  @Override
  public void updateGains(Gains gains) {
    feedback.setPID(gains.getKP(), 0.0, gains.getKD());
    feedforward.setKs(gains.getKS());
    feedforward.setKv(gains.getKV());
    feedforward.setKa(gains.getKA());
  }

  @Override
  public void updateConstraints(AngularPositionConstraints constraints) {
    feedback.setConstraints(
        new TrapezoidProfile.Constraints(
            constraints.maxVelocity().get().in(RadiansPerSecond),
            constraints.maxAcceleration().get().in(RadiansPerSecondPerSecond)));
    feedback.setTolerance(constraints.goalTolerance().get().in(Radian));
  }

  @Override
  public Angle getEncoder1Position() {
    return encoderValue1;
  }

  @Override
  public Angle getEncoder2Position() {
    return encoderValue2;
  }
}
