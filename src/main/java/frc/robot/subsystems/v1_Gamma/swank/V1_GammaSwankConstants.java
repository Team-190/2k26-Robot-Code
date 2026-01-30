package frc.robot.subsystems.v1_Gamma.swank;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;

public class V1_GammaSwankConstants {

  public static final double DRIVE_INERTIA;
  public static final double GEAR_RATIO;
  public static final int MOTOR_CAN_ID;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double STATOR_CURRENT_LIMIT;
  public static final boolean IS_CAN_FD;
  public static final DCMotor MOTOR_CONFIG;
  public static final double MOMENT_OF_INERTIA;

  public static final InvertedValue INVERSION;

  static {
    DRIVE_INERTIA = 0.0;
    GEAR_RATIO = 52.0 / 12.0 * 24.0 / 18.0 * 15.0 / 14.0;
    MOTOR_CAN_ID = 60;
    SUPPLY_CURRENT_LIMIT = 40;
    STATOR_CURRENT_LIMIT = 40;
    IS_CAN_FD = false;
    INVERSION = InvertedValue.CounterClockwise_Positive;
    MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
    MOMENT_OF_INERTIA = 0.067;
  }
}
