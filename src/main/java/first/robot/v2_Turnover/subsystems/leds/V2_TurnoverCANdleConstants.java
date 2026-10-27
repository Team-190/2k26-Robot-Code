package first.robot.v2_Turnover.subsystems.leds;

import com.ctre.phoenix6.CANBus;

public class V2_TurnoverCANdleConstants {

  public static final int CAN_ID = 60;
  public static final CANBus CAN_LOOP = new CANBus();
  public static final int LED_COUNT = 74 - 17; // Not so happy accidents

  public static final double LOW_BATTERY_VOLTAGE = 11.5;
  public static final int MIN_LOOP_CYCLE_COUNT = 20;
}
