package frc.robot.subsystems.shared.turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
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
                constants.MOTOR_CONFIG, constants.MOMENT_OF_INERTIA, constants.GEAR_RATIO),
            constants.MOTOR_CONFIG);

    feedback =
        new ProfiledPIDController(
            constants.GAINS.kP().get(),
            0.0,
            constants.GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                constants.CONSTRAINTS.CRUISING_VELOCITY_RADIANS_PER_SECOND().get(),
                constants.CONSTRAINTS.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED().get()));
    feedback.setTolerance(constants.CONSTRAINTS.GOAL_TOLERANCE_RADIANS().get());

    feedback.disableContinuousInput();

    feedforward =
        new SimpleMotorFeedforward(constants.GAINS.kS().get(), constants.GAINS.kV().get());
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
                constants.TURRET_ANGLE_CALCULATION.GEAR_0_TOOTH_COUNT()
                    / constants.TURRET_ANGLE_CALCULATION.GEAR_1_TOOTH_COUNT())
            .plus(
                // Add some noise
                Degree.of(random.nextDouble() * 0.04 - 0.5));
    encoderValue2 =
        realAngle
            .times(
                constants.TURRET_ANGLE_CALCULATION.GEAR_0_TOOTH_COUNT()
                    / constants.TURRET_ANGLE_CALCULATION.GEAR_2_TOOTH_COUNT())
            .plus(
                // Add some noise
                Degree.of(random.nextDouble() * 0.04 - 0.5));

    inputs.turretAngle = Rotation2d.fromRadians(realAngle.in(Radian));
    inputs.turretVelocityRadiansPerSecond = sim.getAngularVelocityRadPerSec();
    inputs.turretAppliedVolts = appliedVolts;
    inputs.turretSupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.turretGoal = new Rotation2d(positionGoal.getMeasure());
    inputs.turretPositionSetpoint = Rotation2d.fromRadians(feedback.getSetpoint().position);
    inputs.turretPositionError = Rotation2d.fromRadians(feedback.getPositionError());

    inputs.encoder1Position = new Rotation2d(encoderValue1);
    inputs.encoder2Position = new Rotation2d(encoderValue2);
  }

  @Override
  public void setTurretVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void setTurretGoal(Rotation2d goal) {
    double targetGoal = goal.getRadians();
    appliedVolts =
        feedback.calculate(sim.getAngularPositionRad(), targetGoal)
            + feedforward.calculate(feedback.getSetpoint().velocity);
    positionGoal = Rotation2d.fromRadians(targetGoal);
  }

  @Override
  public boolean atTurretPositionGoal() {
    return feedback.atGoal();
  }

  @Override
  public void setPosition(Rotation2d angle) {
    sim.setState(angle.getRadians(), 0.0);
    feedback.reset(angle.getRadians(), 0.0);
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    feedback.setPID(kP, 0.0, kD);
    feedforward.setKs(kS);
    feedforward.setKv(kV);
    feedforward.setKa(kA);
  }

  @Override
  public void updateConstraints(double maxAcceleration, double maxVelocity, double goalTolerance) {
    feedback.setConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    feedback.setTolerance(goalTolerance);
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
