package frc.robot.subsystems.v0_Funky.turret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.InternalLoggedTracer;

public class V0_FunkyTurretIOTalonFX {

  private StatusSignal<Angle> positionRotations;
  private TalonFX talonFX;
  private TalonFXConfiguration config;
  private double positionGoalDegrees;
  private MotionMagicVoltage positionVoltageRequest;
  private VoltageOut voltageRequest;

  public V0_FunkyTurretIOTalonFX() {

    talonFX = new TalonFX(V0_FunkyTurretConstants.CAN_ID);

    config = new TalonFXConfiguration();
    config.Slot0.kP = V0_FunkyTurretConstants.GAINS.kP().get();
    config.Slot0.kD = V0_FunkyTurretConstants.GAINS.kD().get();
    config.Slot0.kV = V0_FunkyTurretConstants.GAINS.kV().get();
    config.Slot0.kA = V0_FunkyTurretConstants.GAINS.kA().get();

    config.CurrentLimits.SupplyCurrentLimit = V0_FunkyTurretConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = V0_FunkyTurretConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        V0_FunkyTurretConstants.MAX_ANGLE * V0_FunkyTurretConstants.GEAR_RATIO;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        V0_FunkyTurretConstants.MIN_ANGLE * V0_FunkyTurretConstants.GEAR_RATIO;

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.MotionMagic.MotionMagicAcceleration =
        V0_FunkyTurretConstants.CONSTRAINTS.MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED().get()
            * V0_FunkyTurretConstants.GEAR_RATIO;
    config.MotionMagic.MotionMagicCruiseVelocity =
        V0_FunkyTurretConstants.CONSTRAINTS.CRUISING_VELOCITY_ROTATIONS_PER_SECOND().get()
            * V0_FunkyTurretConstants.GEAR_RATIO;

    positionRotations = talonFX.getPosition();
    positionVoltageRequest = new MotionMagicVoltage(0);
    voltageRequest = new VoltageOut(0);


  }

  public void setPosition(double angle) {
    InternalLoggedTracer.reset();
    talonFX.setPosition(angle* V0_FunkyTurretConstants.GEAR_RATIO );
    InternalLoggedTracer.record("Set Position", "Turret/TalonFX");
  }

  public void setPositionGoal(double angle) {
    InternalLoggedTracer.reset();
    positionGoalDegrees = angle;
    talonFX.setControl(positionVoltageRequest.withPosition(angle*V0_FunkyTurretConstants.GEAR_RATIO ).withSlot(0));

  }

  public void setTurretVoltage(double volts) {
    InternalLoggedTracer.reset();
    talonFX.setControl(voltageRequest.withOutput(volts));
    InternalLoggedTracer.record("Set Voltage", "Turret/TalonFX");
  }


  
}
