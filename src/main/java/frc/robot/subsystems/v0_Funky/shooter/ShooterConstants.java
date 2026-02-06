package frc.robot.subsystems.v0_Funky.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.CurrentLimits;
import frc.robot.subsystems.shared.turret.TurretConstants;
import frc.robot.subsystems.v0_Funky.V0_FunkyConstants;

public class ShooterConstants {
  public static final int SHOOTER_ID = 30;
  public static final DCMotor SHOOTER_MOTORS = DCMotor.getKrakenX60Foc(2);
  public static final GenericFlywheelConstants SHOOT_CONSTANTS =
      GenericFlywheelConstants.builder()
          .LEADER_CAN_ID(SHOOTER_ID)
          .LEADER_INVERSION(InvertedValue.CounterClockwise_Positive)
          .ENABLE_FOC(true)
          .CURRENT_LIMIT(new CurrentLimits(60.0, 40.0))
          .MOMENT_OF_INERTIA(0.05)
          .GEAR_RATIO(1)
          .MOTOR_CONFIG(SHOOTER_MOTORS)
          .GAINS(
              new GenericFlywheelConstants.Gains(
                  new LoggedTunableNumber("Shooter/Flywheel/Ks", 0),
                  new LoggedTunableNumber("Shooter/Flywheel/Kv", 0),
                  new LoggedTunableNumber("Shooter/Flywheel/Ka", 0),
                  new LoggedTunableNumber("Shooter/Flywheel/Kp", 0),
                  new LoggedTunableNumber("Shooter/Flywheel/Kd", 0)))
          .CONSTRAINTS(
              new GenericFlywheelConstants.Constraints(
                  new LoggedTunableNumber(
                      "Shooter/Flywheel/MaxAccelerationRotationsPerSecondSquared", 6),
                  new LoggedTunableNumber(
                      "Shooter/Flywheel/CruisingVelocityRotationsPerSecondSquared", 4),
                  new LoggedTunableNumber("Shooter/Flywheel/GoalToleranceRadians", 0.05)))
          .OPPOSED_FOLLOWER_CAN_ID(31)
          .build();

  public static final TurretConstants TURRET_CONSTANTS =
      TurretConstants.builder()
          .MOTOR_CONFIG(DCMotor.getKrakenX60Foc(1))
          .MOMENT_OF_INERTIA(0.004)
          .TURRET_CAN_ID(1)
          .CAN_LOOP(V0_FunkyConstants.DRIVE_CONFIG.canBus())
          .LEFT_ENCODER_ID(2)
          .RIGHT_ENCODER_ID(3)
          .MAX_ANGLE(2 * Math.PI)
          .MIN_ANGLE(-2 * Math.PI)
          .GEAR_RATIO(5)
          .SUPPLY_CURRENT_LIMIT(30)
          .STATOR_CURRENT_LIMIT(30)
          .E1_OFFSET_RADIANS(0)
          .E2_OFFSET_RADIANS(0)
          .GAINS(
              new TurretConstants.Gains(
                  new LoggedTunableNumber("Turret/kP", 0),
                  new LoggedTunableNumber("Turret/kD", 0),
                  new LoggedTunableNumber("Turret/kV", 0),
                  new LoggedTunableNumber("Turret/kA", 0),
                  new LoggedTunableNumber("Turret/kS", 0)))
          .CONSTRAINTS(
              new TurretConstants.Constraints(
                  new LoggedTunableNumber("Turret/Max Acceleration", 0),
                  new LoggedTunableNumber("Turret/Cruising Velocity", 0),
                  new LoggedTunableNumber("Turret/Goal Tolerance", 0)))
          .TURRET_ANGLE_CALCULATION(new TurretConstants.TurretAngleCalculation(70, 38, 36))
          .build();
}
