package edu.wpi.team190.gompeilib.core.utility.sysid;

import static org.wpilib.units.Units.*;

import org.wpilib.units.CurrentUnit;
import org.wpilib.units.PerUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.VoltageUnit;

public class CustomUnits {
  public static final PerUnit<CurrentUnit, TimeUnit> ampsPerSecond = Amps.per(Second);
  public static final PerUnit<VoltageUnit, TimeUnit> voltsPerSecond = Volts.per(Second);
}
