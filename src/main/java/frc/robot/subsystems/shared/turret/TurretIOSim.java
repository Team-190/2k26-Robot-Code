package frc.robot.subsystems.shared.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import java.util.Random;

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
            LinearSystemId.createDCMotorSystem(
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
    realAngle = Radian.of(MathUtil.angleModulus((Timer.getFPGATimestamp() * 0.1)));

    encoderValue1 = Angle.ofBaseUnits(0, Radian);
    encoderValue2 = Angle.ofBaseUnits(0, Radian);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
    sim.update(GompeiLib.getLoopPeriod());

    realAngle = Radian.of(sim.getAngularPositionRad());

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
    inputs.velocity = sim.getAngularVelocity();
    inputs.appliedVoltage = Volts.of(appliedVolts);
    inputs.supplyCurrent = Amp.of(sim.getCurrentDrawAmps());
    inputs.positionGoal = positionGoal;
    inputs.positionSetpoint = Rotation2d.fromRadians(feedback.getSetpoint().position);
    inputs.positionError = Rotation2d.fromRadians(feedback.getPositionError());

    inputs.encoder1Position = new Rotation2d(encoderValue1);
    inputs.encoder2Position = new Rotation2d(encoderValue2);
  }

  @Override
  public void setVoltage(Voltage volts) {
    appliedVolts = volts.in(Volt);
  }

  @Override
  public void setPositionGoal(Rotation2d goal) {
    appliedVolts =
        feedback.calculate(sim.getAngularPositionRad(), goal.getRadians())
            + feedforward.calculate(feedback.getSetpoint().velocity);
    positionGoal = goal;
  }

  @Override
  public boolean atPositionGoal(Rotation2d positionReference) {
    return sim.getAngularPositionRad() - positionReference.getRadians()
        <= constants.constraints.goalTolerance().get(Radians);
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
            constraints.maxVelocity().get(RadiansPerSecond),
            constraints.maxAcceleration().get(RadiansPerSecondPerSecond)));
    feedback.setTolerance(constraints.goalTolerance().get(Radian));
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
