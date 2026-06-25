package edu.wpi.team190.gompeilib.core.utility.sysid;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VoltageUnit;

public class CustomUnits {
  public static final PerUnit<CurrentUnit, TimeUnit> ampsPerSecond = Amps.per(Second);
  public static final PerUnit<VoltageUnit, TimeUnit> voltsPerSecond = Volts.per(Second);
}
