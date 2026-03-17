package frc.robot.subsystems.shared.fourbarlinkage;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import lombok.Builder;

@Builder(setterPrefix = "with")
public class FourBarLinkageConstants {

  @Builder.Default public final CANBus canBus = CANBus.roboRIO();

  public final int motorCanId;

  public final int canCoderCanId;
  public final Rotation2d canCoderOffset;
  public final SensorDirectionValue cancoderSensorDirection;

  public final Rotation2d intakeAngleOffset;
  public final Rotation2d zeroOffset;

  public final double gearRatio;
  public final double supplyCurrentLimit;
  public final double statorCurrentLimit;
  public final double momentOfInertia;
  public final DCMotor motorConfig;

  public final Rotation2d minAngle;
  public final Rotation2d maxAngle;

  public final Gains gains;
  public final AngularPositionConstraints constraints;

  public final LinkLengths linkLengths;
  public final LinkBounds linkBounds;

  public final LinkConstants linkConstants;

  public final double pinLength;
  public final Translation3d linkageOffset;

  @Builder.Default public final boolean enableFoc = false;

  public final Rotation2d positionOffsetStep;
  public final Voltage voltageOffsetStep;

  public record LinkLengths(double AB, double BC, double CD, double DA) {}

  public record LinkBounds(double MIN, double PHASE_1, double PHASE_2, double MAX) {}

  public record LinkConstants(double RADIUS_1, double RADIUS_2, double CENTER_OFFSET) {}

  public record LinkageState(Pose3d pose, Rotation2d rotation) {}
}
