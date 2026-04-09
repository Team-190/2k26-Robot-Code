package frc.robot.subsystems.shared.fourbarlinkage;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import lombok.Builder;
import lombok.NonNull;

@Builder(setterPrefix = "with")
public class FourBarLinkageConstants {

  @Builder.Default public final CANBus canBus = CANBus.roboRIO();

  @NonNull public final Integer motorCanId;

  public final Integer canCoderCanId;
  public final Rotation2d canCoderOffset;
  public final SensorDirectionValue cancoderSensorDirection;

  @NonNull public final Rotation2d intakeAngleOffset;
  @NonNull public final Rotation2d zeroOffset;

  @NonNull public final Double gearRatio;
  @NonNull public final CurrentLimits currentLimits;
  @NonNull public final Double momentOfInertia;
  @NonNull public final DCMotor motorConfig;

  @NonNull public final Rotation2d minAngle;
  @NonNull public final Rotation2d maxAngle;
  @NonNull public final Rotation2d startAngle;

  @NonNull public final Gains gains;
  @NonNull public final AngularPositionConstraints constraints;

  @NonNull public final LinkLengths linkLengths;
  @NonNull public final LinkBounds linkBounds;

  @NonNull public final LinkConstants linkConstants;

  @NonNull public final Double pinLength;
  @NonNull public final Translation3d linkageOffset;

  @NonNull public final Boolean enableFoc;

  public final Rotation2d positionOffsetStep;
  public final Voltage voltageOffsetStep;

  public record LinkLengths(double AB, double BC, double CD, double DA) {}

  public record LinkBounds(double MIN, double PHASE_1, double PHASE_2, double MAX) {}

  public record LinkConstants(double RADIUS_1, double RADIUS_2, double CENTER_OFFSET) {}

  public record LinkageState(Pose3d pose, Rotation2d rotation) {}
}
