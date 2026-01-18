package frc.robot.subsystems.v0_Funky.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;

public class V0_FunkyTurretIOSim implements V0_FunkyTurretIO {

  private ProfiledPIDController feedback;
  private SimpleMotorFeedforward feedforward;

  private double e1;
  private double e2;

  private double directionalGoalRadians;

  private DCMotorSim sim;

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

    feedforward =
        new SimpleMotorFeedforward(
            V0_FunkyTurretConstants.GAINS.kS().get(), V0_FunkyTurretConstants.GAINS.kV().get());
  }

  @Override
  public void updateInputs(V0_FunkyTurretIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(GompeiLib.getLoopPeriod());

    inputs.turretAngle = Rotation2d.fromRadians(sim.getAngularPositionRotations());
    inputs.turretVelocityRadiansPerSecond = sim.getAngularVelocityRadPerSec();
    inputs.turretAppliedVolts = appliedVolts;
    inputs.turretSupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.turretGoal = positionGoal.getRadians();
    inputs.turretPositionSetpoint = feedback.getSetpoint().position;
    inputs.turretPositionError = feedback.getPositionError();
  }

  @Override
  public void setTurretVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void setTurretGoal(Pose3d goal) {
    positionGoal = goal.toPose2d().getRotation();
  }

  @Override
  public void setTurretGoal(double goal) {
    double directionalGoalRadians = 0;
    double positiveDiff = goal - V0_FunkyTurretIOTalonFX.calculateTurretAngle(e1, e2);
    double negativeDiff = positiveDiff - 2 * Math.PI;
    if (Math.abs(positiveDiff) < Math.abs(negativeDiff)
        && goal <= V0_FunkyTurretConstants.MAX_ANGLE
        && goal >= V0_FunkyTurretConstants.MIN_ANGLE) {
      directionalGoalRadians = positiveDiff;
    } else if ((goal - 2 * Math.PI) <= V0_FunkyTurretConstants.MAX_ANGLE
        && (goal - 2 * Math.PI) >= V0_FunkyTurretConstants.MIN_ANGLE) {
      directionalGoalRadians = negativeDiff;
    }
  }

  @Override
  public void stopTurret() {
    setTurretVoltage(0);
  }

  @Override
  public boolean atTurretPositionGoal() {
    return feedback.atGoal();
  }

  @Override
  public void setPosition(double angle) {
    setPosition(angle * V0_FunkyTurretConstants.GEAR_RATIO);
  }

  @Override
  public void goToZero() {
    positionGoal = new Rotation2d(0.0);
  }

  @Override
  public void checkDirectionalMotion() {
    if (positionGoal.getRadians() < V0_FunkyTurretConstants.MIN_ANGLE) {
      appliedVolts = (directionalGoalRadians * V0_FunkyTurretConstants.GEAR_RATIO / (2 * Math.PI));
    } else if (positionGoal.getRadians() > V0_FunkyTurretConstants.MAX_ANGLE) {
      appliedVolts =
          -1 * (directionalGoalRadians * V0_FunkyTurretConstants.GEAR_RATIO / (2 * Math.PI));
    }
  }
}
