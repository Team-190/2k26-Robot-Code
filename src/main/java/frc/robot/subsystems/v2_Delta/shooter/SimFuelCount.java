package frc.robot.subsystems.v2_Delta.shooter;
import lombok.Getter;
import lombok.Setter;

public class SimFuelCount {
    @Getter private static final int capacity = 35;
    @Getter private static final double launchBPS = 15.0; // update accordingly

    @Setter @Getter private int fuelStored;

    public SimFuelCount(int fuelStored) {
      this.fuelStored = fuelStored;
    }
}