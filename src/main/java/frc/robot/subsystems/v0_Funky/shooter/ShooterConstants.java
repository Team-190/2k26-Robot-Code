package frc.robot.subsystems.v0_Funky.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.Constraints;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.Gains;

public class ShooterConstants {
  public static final GenericFlywheelConstants SHOOTER_FLYWHEEL_CONSTANTS =
      new GenericFlywheelConstants(
          10,
          1,
          false,
          false,
          40.0,
          0.004,
          new Gains(
              new LoggedTunableNumber("Ks", 0),
              new LoggedTunableNumber("Kv", 0),
              new LoggedTunableNumber("Ka", 0),
              new LoggedTunableNumber("Kp", 0),
              new LoggedTunableNumber("Kd", 0)),
          DCMotor.getKrakenX60Foc(1),
          new int[] {1},
          new int[] {},
          new Constraints(
              new LoggedTunableNumber("max accel", 0),
              new LoggedTunableNumber("max vel", 0),
              new LoggedTunableNumber("tolerance", 1)),
          1,
          InvertedValue.CounterClockwise_Positive);
}
