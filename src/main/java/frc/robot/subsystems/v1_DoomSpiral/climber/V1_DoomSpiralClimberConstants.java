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

  public static final CurrentLimits CURRENT_LIMITS =
      CurrentLimits.builder()
          .withArmSupplyCurrentLimit(40.0)
          .withArmStatorCurrentLimit(60.0)
          .withArmTorqueCurrentLimit(40.0)
          .build();

  public static final double MOMENT_OF_INERTIA_KG_M2 = 0.0817231996;
  public static boolean ENABLE_FOC = false;

  public static final ArmParameters ARM_PARAMETERS =
      ArmParameters.builder()
          .withMotorConfig(DCMotor.getKrakenX60Foc(1))
          .withMinAngle(new Rotation2d())
          .withMaxAngle(new Rotation2d())
          .withNumMotors(1)
          .withGearRatio(165.0)
          .withLengthMeters(0.259)
          .withContinuousOutput(false)
          .withMomentOfInertia(MOMENT_OF_INERTIA_KG_M2)
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
          .withArmCANID(MOTOR_CAN_ID)
          //   .with(V1_DoomSpiralConstants.DRIVE_CONFIG.canBus())
          .withArmParameters(ARM_PARAMETERS)
          .withSlot0Gains(SLOT_0_GAINS)
          .withConstraints(CONSTRAINTS)
          .withCurrentLimits(CURRENT_LIMITS)
          .withEnableFOC(ENABLE_FOC)
          .build();
}
