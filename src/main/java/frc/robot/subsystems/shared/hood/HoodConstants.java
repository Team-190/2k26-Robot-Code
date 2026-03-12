package frc.robot.subsystems.shared.hood;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import lombok.Builder;
import lombok.NonNull;

@Builder(setterPrefix = "with")
public class HoodConstants {
  @NonNull public final Integer motorCanId;
  @NonNull public final CANBus canBus;

  @NonNull public final Double gearRatio;
  @NonNull public final Double currentLimits;
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
