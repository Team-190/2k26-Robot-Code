package frc.robot.subsystems.v1_Gamma.spindexer;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;

public class V1_GammaSpindexerConstants {
  public static final int MOTOR_CAN_ID;
  public static final double GEAR_RATIO;
  public static final double STATOR_CURRENT_LIMIT;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final MomentOfInertia MOMENT_OF_INERTIA;
  public static final DCMotor MOTOR_CONFIG;
  public static final InvertedValue SPINDEXER_INVERTED_VALUE;

  static {
    MOTOR_CAN_ID = 0;
    GEAR_RATIO = 0;
    STATOR_CURRENT_LIMIT = 0;
    SUPPLY_CURRENT_LIMIT = 0;
    MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1); //TODO: Replace with actual motor.
    MOMENT_OF_INERTIA = KilogramSquareMeters.of(0); 
    SPINDEXER_INVERTED_VALUE = InvertedValue.Clockwise_Positive; //TODO: Replace with actual value.
        
 } 
  
}
