package edu.wpi.team190.gompeilib.subsystems.generic.hood;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import lombok.Builder;
import lombok.NonNull;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

@Builder(setterPrefix = "with")
public class GenericHoodConstants {
  @NonNull public final Integer motorCanId;
  @NonNull public final CANBus canBus;

  @NonNull public final Double gearRatio;
  @NonNull public final CurrentLimits currentLimits;
  @NonNull public final Double momentOfInertia;
  @NonNull public final InvertedValue invertedValue;
  @NonNull public final DCMotor motorConfig;
  @NonNull public final Double lengthMeters;
  @NonNull public final Rotation2d minAngle;
  @NonNull public final Rotation2d maxAngle;

  @NonNull public final Voltage zeroVoltage;
  @NonNull public final Current zeroCurrentThreshold;
  @NonNull public final Current zeroCurrentEpsilon;

  @NonNull public final Gains gains;
  @NonNull public final AngularPositionConstraints constraints;

  @NonNull public final Voltage voltageStep;
  @NonNull public final Angle offsetStep;
}
