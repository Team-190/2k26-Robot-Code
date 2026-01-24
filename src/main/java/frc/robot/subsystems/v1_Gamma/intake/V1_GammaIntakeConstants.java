package frc.robot.subsystems.v1_Gamma.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.Constraints;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.Gains;

public class V1_GammaIntakeConstants {
  public static final int[] CAN_IDS_TOP = {16};
  public static final boolean ON_CANIVORE_TOP = true;
  public static final boolean ENABLE_FOC_TOP = false;
  public static final double CURRENT_LIMIT_TOP = 30.0;
  public static final double MOMENT_OF_INERTIA_TOP = 0.0;
  public static final double GEAR_RATIO_TOP = 1.0;
  public static final DCMotor MOTOR_CONFIG_TOP = new DCMotor(0, 0, 0, 0, 0, 0);
  public static final Gains GAINS_TOP =
      new Gains(
          new LoggedTunableNumber("Ks", 0),
          new LoggedTunableNumber("Kv", 0),
          new LoggedTunableNumber("Ka", 0),
          new LoggedTunableNumber("Kp", 0),
          new LoggedTunableNumber("Kd", 0));
  public static final Constraints CONSTRAINTS_TOP =
      new Constraints(
          new LoggedTunableNumber("MaxAccelerationRotationsPerSecondSquared", 0),
          new LoggedTunableNumber("CruisingVelocityRotationsPerSecond", 0),
          new LoggedTunableNumber("GoalToleranceRotationsPerSecond", 0));
  public static final InvertedValue MOTOR_INVERTED_TOP = InvertedValue.Clockwise_Positive;

  public static final GenericFlywheelConstants INTAKE_FLYWHEEL_CONSTANTS_TOP =
      new GenericFlywheelConstants(
          CAN_IDS_TOP,
          ON_CANIVORE_TOP,
          ENABLE_FOC_TOP,
          CURRENT_LIMIT_TOP,
          MOMENT_OF_INERTIA_TOP,
          GAINS_TOP,
          MOTOR_CONFIG_TOP,
          CONSTRAINTS_TOP,
          GEAR_RATIO_TOP,
          MOTOR_INVERTED_TOP);

  public static final int[] CAN_IDS_BOTTOM = {16};
  public static final boolean ON_CANIVORE_BOTTOM = true;
  public static final boolean ENABLE_FOC_BOTTOM = false;
  public static final double CURRENT_LIMIT_BOTTOM = 30.0;
  public static final double MOMENT_OF_INERTIA_BOTTOM = 0.0;
  public static final double GEAR_RATIO_BOTTOM = 1.0;
  public static final DCMotor MOTOR_CONFIG_BOTTOM = new DCMotor(0, 0, 0, 0, 0, 0);
  public static final Gains GAINS_BOTTOM =
      new Gains(
          new LoggedTunableNumber("Ks", 0),
          new LoggedTunableNumber("Kv", 0),
          new LoggedTunableNumber("Ka", 0),
          new LoggedTunableNumber("Kp", 0),
          new LoggedTunableNumber("Kd", 0));
  public static final Constraints CONSTRAINTS_BOTTOM =
      new Constraints(
          new LoggedTunableNumber("MaxAccelerationRotationsPerSecondSquared", 0),
          new LoggedTunableNumber("CruisingVelocityRotationsPerSecond", 0),
          new LoggedTunableNumber("GoalToleranceRotationsPerSecond", 0));
  public static final InvertedValue MOTOR_INVERTED_BOTTOM = InvertedValue.Clockwise_Positive;

  public static final GenericFlywheelConstants INTAKE_FLYWHEEL_CONSTANTS_BOTTOM =
      new GenericFlywheelConstants(
          CAN_IDS_BOTTOM,
          ON_CANIVORE_BOTTOM,
          ENABLE_FOC_BOTTOM,
          CURRENT_LIMIT_BOTTOM,
          MOMENT_OF_INERTIA_BOTTOM,
          GAINS_BOTTOM,
          MOTOR_CONFIG_BOTTOM,
          CONSTRAINTS_BOTTOM,
          GEAR_RATIO_BOTTOM,
          MOTOR_INVERTED_BOTTOM);
}
