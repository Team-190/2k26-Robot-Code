package frc.robot.subsystems.v1_Gamma.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.ArmParameters;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.Gains;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.Constraints;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.CurrentLimits;


public class V1_GammaClimberConstants {
  public static final ArmConstants CLIMBER_CONSTANTS = new ArmConstants(
      42,
      new ArmParameters(
          new DCMotor(0, 0, 0, 0, 0, 0),
          new Rotation2d(),
          new Rotation2d(),
          1,
          12,
          34.0,
          45.0,
          56.0),
      new Gains(
          new LoggedTunableNumber("Ks", 0),
          new LoggedTunableNumber("Kv", 0),
          new LoggedTunableNumber("Ka", 0),
          new LoggedTunableNumber("Kg", 0),
          new LoggedTunableNumber("Kp", 0),
          new LoggedTunableNumber("Kd", 0)),
      new Gains(
          new LoggedTunableNumber("Ks", 0),
          new LoggedTunableNumber("Kv", 0),
          new LoggedTunableNumber("Ka", 0),
          new LoggedTunableNumber("Kg", 0),
          new LoggedTunableNumber("Kp", 0),
          new LoggedTunableNumber("Kd", 0)),
      new Gains(
          new LoggedTunableNumber("Ks", 0),
          new LoggedTunableNumber("Kv", 0),
          new LoggedTunableNumber("Ka", 0),
          new LoggedTunableNumber("Kg", 0),
          new LoggedTunableNumber("Kp", 0),
          new LoggedTunableNumber("Kd", 0)),
      new Constraints(
          new LoggedTunableNumber("MaxAccelerationRotationsPerSecondSquared", 0),
          new LoggedTunableNumber("CruisingVelocityRotationsPerSecondSquared", 0),
          new LoggedTunableNumber("GoalToleranceRadians", 0)),
      new CurrentLimits(0, 0, 0),
      1.0,
      false);

}
