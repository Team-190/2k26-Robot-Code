package frc.robot.subsystems.shared.fourbarlinkage;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import lombok.Builder;

@Builder
public class FourBarLinkageConstants {

  @Builder.Default public final CANBus CAN_LOOP = new CANBus();

  public final int MOTOR_CAN_ID;

  public final int CAN_CODER_CAN_ID;
  public final SensorDirectionValue CANCODER_SENSOR_DIRECTION;

  public final Rotation2d INTAKE_ANGLE_OFFSET;
  public final Rotation2d ZERO_OFFSET;

  public final double GEAR_RATIO;
  public final double SUPPLY_CURRENT_LIMIT;
  public final double STATOR_CURRENT_LIMIT;
  public final double MOMENT_OF_INERTIA;
  public final DCMotor MOTOR_CONFIG;

  public final Rotation2d MIN_ANGLE;
  public final Rotation2d MAX_ANGLE;

  public final Gains GAINS;
  public final Constraints CONSTRAINTS;

  public final LinkLengths LINK_LENGTHS;
  public final LinkBounds LINK_BOUNDS;

  public final LinkConstants LINK_CONST;

  public final double PIN_LENGTH;
  public final Translation3d LINKAGE_OFFSET;

  @Builder.Default public final boolean ENABLE_FOC = false;

  public record Gains(
      LoggedTunableNumber kp,
      LoggedTunableNumber kd,
      LoggedTunableNumber ks,
      LoggedTunableNumber kv,
      LoggedTunableNumber ka) {}

  public record Constraints(
      LoggedTunableNumber maxVelocityRadiansPerSecond,
      LoggedTunableNumber maxAccelerationRadiansPerSecondSqaured,
      LoggedTunableNumber goalToleranceRadians) {}

  public record LinkLengths(double AB, double BC, double CD, double DA) {}

  public record LinkBounds(double MIN, double PHASE_1, double PHASE_2, double MAX) {}

  public record LinkConstants(double RADIUS_1, double RADIUS_2, double CENTER_OFFSET) {}

  public enum LinkageGoal {
    SCORE,
    FEED,
    STOW
  }

  public record LinkageState(Pose3d pose, Rotation2d rotation) {}
}
