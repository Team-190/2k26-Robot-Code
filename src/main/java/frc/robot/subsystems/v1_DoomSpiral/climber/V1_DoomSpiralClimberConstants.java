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
  public static final Gains SLOT_0_GAINS =
      Gains.builder()
          .withKP(new LoggedTunableNumber("Climber/Slot0/kP", 0))
          .withKD(new LoggedTunableNumber("Climber/Slot0/kD", 0))
          .withKS(new LoggedTunableNumber("Climber/Slot0/kS", 0))
          .withKG(new LoggedTunableNumber("Climber/Slot0/kG", 0))
          .withKV(new LoggedTunableNumber("Climber/Slot0/kV", 0))
          .withKA(new LoggedTunableNumber("Climber/Slot0/kA", 0))
          .build();

  public static final Constraints CONSTRAINTS =
      Constraints.builder()
          .withMaxAccelerationRadiansPerSecondSquared(
              new LoggedTunableNumber("Climber/MaxAccelerationRotationsPerSecondSquared", 6))
          .withCruisingVelocityRadiansPerSecond(
              new LoggedTunableNumber("Climber/CruisingVelocityRotationsPerSecondSquared", 4))
          .withGoalToleranceRadians(new LoggedTunableNumber("Climber/GoalToleranceRadians", 0.05))
          .build();

  public static final CurrentLimits CURRENT_LIMITS =
      CurrentLimits.builder()
          .withArmSupplyCurrentLimit(40.0)
          .withArmStatorCurrentLimit(60.0)
          .withArmTorqueCurrentLimit(40.0)
          .build();

  public static boolean ENABLE_FOC = false;

  public static final ArmParameters ARM_PARAMETERS =
      ArmParameters.builder()
          .withMotorConfig(DCMotor.getKrakenX60Foc(1))
          .withMinAngle(Rotation2d.fromRadians(-100 * Math.PI))
          .withMaxAngle(Rotation2d.fromRadians(100 * Math.PI))
          .withNumMotors(1)
          .withGearRatio(165.0)
          .withLengthMeters(0.259)
          .withContinuousOutput(false)
          .withMomentOfInertia(0.0817231996)
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
          .withArmCANID(60)
          .withArmParameters(ARM_PARAMETERS)
          .withSlot0Gains(SLOT_0_GAINS)
          .withConstraints(CONSTRAINTS)
          .withCurrentLimits(CURRENT_LIMITS)
          .withEnableFOC(ENABLE_FOC)
          .build();
}
