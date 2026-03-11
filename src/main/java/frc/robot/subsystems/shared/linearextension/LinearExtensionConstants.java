package frc.robot.subsystems.shared.linearextension;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.LinearConstraints;
import lombok.Builder;

@Builder
public class LinearExtensionConstants {

  @Builder.Default public final CANBus CAN_LOOP = CANBus.roboRIO();

  public final int MOTOR_CAN_ID;

  public final int CAN_CODER_CAN_ID;
  public final Rotation2d CAN_CODER_OFFSET;
  public final SensorDirectionValue CANCODER_SENSOR_DIRECTION;

  public final double GEAR_RATIO;
  public final double SUPPLY_CURRENT_LIMIT;
  public final double STATOR_CURRENT_LIMIT;
  public final double MOMENT_OF_INERTIA;
  public final DCMotor MOTOR_CONFIG;

  public final Gains GAINS;
  public final LinearConstraints CONSTRAINTS;

  @Builder.Default public final boolean ENABLE_FOC = false;
}
