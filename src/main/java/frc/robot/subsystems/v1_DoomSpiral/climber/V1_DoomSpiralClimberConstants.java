package frc.robot.subsystems.v1_DoomSpiral.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.ArmParameters;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.Constraints;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.CurrentLimits;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.Gains;
import lombok.AllArgsConstructor;
import lombok.Getter;

public class V1_DoomSpiralClimberConstants {
  public static final int MOTOR_CAN_ID = 15;
  public static final ArmParameters ARM_PARAMETERS =
      new ArmParameters(
          DCMotor.getKrakenX60Foc(1),
          new Rotation2d(-Math.PI * 4),
          new Rotation2d(Math.PI * 4),
          1,
          12,
          3.0,
          4.0,
          5.0);
  public static final Gains SLOT_1_GAINS =
      new Gains(
          new LoggedTunableNumber("Ks", 0),
          new LoggedTunableNumber("Kv", 0),
          new LoggedTunableNumber("Ka", 0),
          new LoggedTunableNumber("Kg", 0),
          new LoggedTunableNumber("Kp", 0),
          new LoggedTunableNumber("Kd", 0));
  public static final Gains SLOT_2_GAINS =
      new Gains(
          new LoggedTunableNumber("Ks", 0),
          new LoggedTunableNumber("Kv", 0),
          new LoggedTunableNumber("Ka", 0),
          new LoggedTunableNumber("Kg", 0),
          new LoggedTunableNumber("Kp", 0),
          new LoggedTunableNumber("Kd", 0));
  public static final Gains SLOT_3_GAINS =
      new Gains(
          new LoggedTunableNumber("Ks", 0),
          new LoggedTunableNumber("Kv", 0),
          new LoggedTunableNumber("Ka", 0),
          new LoggedTunableNumber("Kg", 0),
          new LoggedTunableNumber("Kp", 0),
          new LoggedTunableNumber("Kd", 0));

  public static final Constraints CONSTRAINTS =
      new Constraints(
          new LoggedTunableNumber("MaxAccelerationRotationsPerSecondSquared", 6),
          new LoggedTunableNumber("CruisingVelocityRotationsPerSecondSquared", 4),
          new LoggedTunableNumber("GoalToleranceRadians", 0.05));

  public static final CurrentLimits CURRENT_LIMITS = new CurrentLimits(40, 60, 1000);

  public static final double MOMENT_OF_INERTIA_KG_M2 = 0.05;
  public static boolean ENABLE_FOC = false;

  public static final double positionToleranceRadians = 0.01;

  @AllArgsConstructor
  public static enum ClimberGoal {
    L1_POSITION_GOAL(new Rotation2d(0)),
    L1_AUTO_POSITION_GOAL(new Rotation2d(0)),
    L2_POSITION_GOAL(new Rotation2d(0)),
    L2_FLIP_GOAL(new Rotation2d(0)),
    DEFAULT(new Rotation2d(0));

    @Getter private Rotation2d position;
  }

  public static final ArmConstants CLIMBER_CONSTANTS =
      new ArmConstants(
          MOTOR_CAN_ID,
          ARM_PARAMETERS,
          SLOT_1_GAINS,
          SLOT_2_GAINS,
          SLOT_3_GAINS,
          CONSTRAINTS,
          CURRENT_LIMITS,
          MOMENT_OF_INERTIA_KG_M2,
          ENABLE_FOC);
}
