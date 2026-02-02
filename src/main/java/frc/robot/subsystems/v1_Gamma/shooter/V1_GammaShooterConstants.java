package frc.robot.subsystems.v1_Gamma.shooter;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;

public class V1_GammaShooterConstants {
    GenericFlywheelConstants SHOOTER_FLYWHEEL_CONSTANTS =
        new GenericFlywheelConstants(
            190,
            1,
            false,
            false,
            40.0,
            0.004,
            new GenericFlywheelConstants.Gains(
                new LoggedTunableNumber("Ks", 0),
                new LoggedTunableNumber("Kv", 0),
                new LoggedTunableNumber("Ka", 0),
                new LoggedTunableNumber("Kp", 0),
                new LoggedTunableNumber("Kd", 0)),
            DCMotor.getKrakenX60Foc(1),
            new int[] {30},
            new int[] {31},
            new GenericFlywheelConstants.Constraints(
                new LoggedTunableNumber("max accel", 0),
                new LoggedTunableNumber("max vel", 0),
                new LoggedTunableNumber("tolerance", 1)),
            1,
            InvertedValue.CounterClockwise_Positive);
}
