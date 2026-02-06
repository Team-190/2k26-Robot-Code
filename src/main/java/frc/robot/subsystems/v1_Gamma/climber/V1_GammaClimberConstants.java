package frc.robot.subsystems.v1_Gamma.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.ArmParameters;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.Constraints;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.CurrentLimits;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.Gains;
import frc.robot.subsystems.v1_Gamma.V1_GammaConstants;
import lombok.AllArgsConstructor;
import lombok.Getter;

public class V1_GammaClimberConstants {
  public static final int MOTOR_CAN_ID = 60;
  public static final Gains SLOT_0_GAINS =
      new Gains(
          new LoggedTunableNumber("Climber/Slot0/kP", 0),
          new LoggedTunableNumber("Climber/Slot0/kD", 0),
          new LoggedTunableNumber("Climber/Slot0/kS", 0),
          new LoggedTunableNumber("Climber/Slot0/kG", 0),
          new LoggedTunableNumber("Climber/Slot0/kV", 0),
          new LoggedTunableNumber("Climber/Slot0/kA", 0));

  public static final Constraints CONSTRAINTS =
      new Constraints(
          new LoggedTunableNumber("Climber/MaxAccelerationRotationsPerSecondSquared", 6),
          new LoggedTunableNumber("Climber/CruisingVelocityRotationsPerSecondSquared", 4),
          new LoggedTunableNumber("Climber/GoalToleranceRadians", 0.05));

  public static final CurrentLimits CURRENT_LIMITS = new CurrentLimits(40, 60, 0);

  public static final double MOMENT_OF_INERTIA_KG_M2 = 0.0817231996;
  public static boolean ENABLE_FOC = false;

  public static final ArmParameters ARM_PARAMETERS =
      //      new ArmParameters(
      //          DCMotor.getKrakenX60(1),
      //          new Rotation2d(Double.NEGATIVE_INFINITY),
      //          new Rotation2d(Double.POSITIVE_INFINITY),
      //          1,
      //          165,
      //          0.259,
      //          0,
      //          0);
      ArmParameters.builder()
          .MOTOR_CONFIG(DCMotor.getKrakenX60Foc(1))
          .MIN_ANGLE(new Rotation2d())
          .MAX_ANGLE(new Rotation2d())
          .NUM_MOTORS(1)
          .GEAR_RATIO(165)
          .LENGTH_METERS(0.259)
          .MOMENT_OF_INERTIA(MOMENT_OF_INERTIA_KG_M2)
          .build();

  @AllArgsConstructor
  public enum ClimberGoal {
    L1_POSITION_GOAL(new Rotation2d(0)),
    L1_AUTO_POSITION_GOAL(new Rotation2d(0)),
    L2_POSITION_GOAL(new Rotation2d(0)),
    L2_FLIP_GOAL(new Rotation2d(0)),
    DEFAULT(new Rotation2d(0));

    @Getter private final Rotation2d position;
  }

  public static final ArmConstants CLIMBER_CONSTANTS =
      ArmConstants.builder()
          .ARM_CAN_ID(MOTOR_CAN_ID)
          .CAN_LOOP(V1_GammaConstants.DRIVE_CONFIG.canBus())
          .ARM_PARAMETERS(ARM_PARAMETERS)
          .SLOT0_GAINS(SLOT_0_GAINS)
          .CONSTRAINTS(CONSTRAINTS)
          .CURRENT_LIMITS(CURRENT_LIMITS)
          .ENABLE_FOC(ENABLE_FOC)
          .build();
}
