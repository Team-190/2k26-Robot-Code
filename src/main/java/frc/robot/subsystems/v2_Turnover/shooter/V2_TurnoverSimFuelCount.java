package frc.robot.subsystems.v2_Turnover.shooter;

import lombok.Getter;
import lombok.Setter;

public class V2_TurnoverSimFuelCount {
  @Getter private static final int capacity = 35;
  @Getter private static final double launchBPS = 15.0; // update accordingly

  @Setter @Getter private int fuelStored;

  public V2_TurnoverSimFuelCount(int fuelStored) {
    this.fuelStored = fuelStored;
  }
}
