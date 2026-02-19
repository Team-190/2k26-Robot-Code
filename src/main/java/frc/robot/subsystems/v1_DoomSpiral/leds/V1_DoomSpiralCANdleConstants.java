package frc.robot.subsystems.v1_DoomSpiral.leds;

import com.ctre.phoenix6.CANBus;

public class V1_DoomSpiralCANdleConstants {

  public static final int CAN_ID = 51;
  public static final CANBus CAN_LOOP = CANBus.roboRIO();
  public static final int LED_COUNT = 50;

  public static final double LOW_BATTERY_VOLTAGE = 11.5;
  public static final int MIN_LOOP_CYCLE_COUNT = 10;
}
