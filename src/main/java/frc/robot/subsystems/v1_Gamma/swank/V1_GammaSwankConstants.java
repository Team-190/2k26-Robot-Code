package frc.robot.subsystems.v1_Gamma.swank;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.system.plant.DCMotor;

public class V1_GammaSwankConstants {

  public static final double MOMENT_OF_INERTIA = 0.004;
  public static final double GEAR_RATIO = 52.0 / 12.0 * 24.0 / 18.0 * 15.0 / 14.0;
  public static final int MOTOR_CAN_ID = -1;
  public static final double SUPPLY_CURRENT_LIMIT = 40;
  public static final double STATOR_CURRENT_LIMIT = 40;
  public static final boolean IS_CAN_FD = false;

  public static final InvertedValue INVERSION = InvertedValue.CounterClockwise_Positive;

  public static final DCMotor MOTOR_CONFIG = DCMotor.getKrakenX44Foc(1);
}
