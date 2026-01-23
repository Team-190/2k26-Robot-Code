package frc.robot.subsystems.v1_Gamma.climber;

import edu.wpi.first.math.system.plant.DCMotor;

public class V1_GammaClimberConstants {
  public static final int MOTOR_CAN_ID = -1; // TODO: Put real data

  public static final double SUPPLY_CURRENT_LIMIT = -1; // TODO: Put real data
  public static final double STATOR_CURRENT_LIMIT = -1; // TODO: Put real data

  public static final double CLIMBER_CLIMBED_L1_RADIANS = 67; // TODO: Put real data
  public static final double CLIMBER_CLIMBED_L2_RADIANS = -67; // TODO: Put real data

  public static final double GEAR_RATIO = -1; // TODO: Put real data
  public static final DCMotor MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1); // TODO: Put real data

  public static final boolean IS_CAN_FD = false;
}
