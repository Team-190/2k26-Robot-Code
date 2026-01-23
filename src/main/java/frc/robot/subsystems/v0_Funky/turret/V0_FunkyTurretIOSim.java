package frc.robot.subsystems.v0_Funky.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

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
import org.littletonrobotics.junction.Logger;

public class V0_FunkyTurretIOSim implements V0_FunkyTurretIO {

  private final ProfiledPIDController feedback;
  private final SimpleMotorFeedforward feedforward;

  private final Random random = new Random();

  private final DCMotorSim sim;

  private Rotation2d positionGoal = new Rotation2d();
  private double appliedVolts = 0.0;

  private Angle encoderValue1;
  private Angle encoderValue2;
  private Angle realAngle;

  public V0_FunkyTurretIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V0_FunkyTurretConstants.MOTOR_CONFIG,
                V0_FunkyTurretConstants.MOMENT_OF_INERTIA,
                V0_FunkyTurretConstants.GEAR_RATIO),
            V0_FunkyTurretConstants.MOTOR_CONFIG);

    feedback =
        new ProfiledPIDController(
            V0_FunkyTurretConstants.GAINS.kP().get(),
            0.0,
            V0_FunkyTurretConstants.GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                V0_FunkyTurretConstants.CONSTRAINTS.CRUISING_VELOCITY_RADIANS_PER_SECOND().get(),
                V0_FunkyTurretConstants.CONSTRAINTS
                    .MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED()
                    .get()));
    feedback.setTolerance(V0_FunkyTurretConstants.CONSTRAINTS.GOAL_TOLERANCE_RADIANS().get());

    feedback.disableContinuousInput();

    feedforward =
        new SimpleMotorFeedforward(
            V0_FunkyTurretConstants.GAINS.kS().get(), V0_FunkyTurretConstants.GAINS.kV().get());

    encoderValue1 = Angle.ofBaseUnits(random.nextDouble(0, 2 * Math.PI), Radians);
    encoderValue2 = Angle.ofBaseUnits(random.nextDouble(0, 2 * Math.PI), Radians);
  }

  @Override
  public void updateInputs(V0_FunkyTurretIOInputs inputs) {
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
    sim.update(GompeiLib.getLoopPeriod());

    encoderValue1 = encoderValue1.plus(Degrees.one());
    encoderValue2 = encoderValue2.plus(Degrees.one());
    realAngle = Degrees.of((Timer.getFPGATimestamp() / 360) % 720);
    Logger.recordOutput("Turret/Sim/Real Angle Degrees", realAngle);
    System.out.println("Turret Real Angle: " + realAngle);

    inputs.turretAngle = Rotation2d.fromRadians(sim.getAngularPositionRad());
    inputs.turretVelocityRadiansPerSecond = sim.getAngularVelocityRadPerSec();
    inputs.turretAppliedVolts = appliedVolts;
    inputs.turretSupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.turretGoal = new Rotation2d(positionGoal.getMeasure());
    inputs.turretPositionSetpoint = Rotation2d.fromRadians(feedback.getSetpoint().position);
    inputs.turretPositionError = Rotation2d.fromRadians(feedback.getPositionError());
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
  public Angle getE1() {
    return encoderValue1;
  }

  @Override
  public Angle getE2() {
    return encoderValue2;
  }
}
