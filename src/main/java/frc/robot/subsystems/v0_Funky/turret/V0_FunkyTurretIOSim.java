package frc.robot.subsystems.v0_Funky.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;

public class V0_FunkyTurretIOSim implements V0_FunkyTurretIO {

  private final ProfiledPIDController feedback;
  private final SimpleMotorFeedforward feedforward;

  private final DCMotorSim sim;

  private Rotation2d positionGoal = new Rotation2d();
  private double appliedVolts = 0.0;

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
  }

  @Override
  public void updateInputs(V0_FunkyTurretIOInputs inputs) {
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
    sim.update(GompeiLib.getLoopPeriod());

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

    while (targetGoal < V0_FunkyTurretConstants.MIN_ANGLE) {
      targetGoal += 2 * Math.PI;
    }
    while (targetGoal > V0_FunkyTurretConstants.MAX_ANGLE) {
      targetGoal -= 2 * Math.PI;
    }

    targetGoal =
        MathUtil.clamp(
            targetGoal, V0_FunkyTurretConstants.MIN_ANGLE, V0_FunkyTurretConstants.MAX_ANGLE);

    appliedVolts =
        feedback.calculate(sim.getAngularPositionRad(), targetGoal)
            + feedforward.calculate(feedback.getSetpoint().velocity);
    positionGoal = Rotation2d.fromRadians(targetGoal);
  }

  @Override
  public void incrementAngle(Rotation2d increment) {
    setTurretGoal(increment.plus(new Rotation2d(sim.getAngularPositionRad())));
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
}
