package frc.robot.subsystems.v1_DoomSpiral.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.Constraints;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.CurrentLimits;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.Gains;
import frc.robot.subsystems.shared.hood.HoodConstants;
import java.util.Set;

public class V1_DoomSpiralShooterConstants {

  public static final CurrentLimits currentLimits =
      CurrentLimits.builder().withStatorCurrentLimit(40.0).withSupplyCurrentLimit(40.0).build();

  public static final Gains flywheelGains =
      Gains.builder().withKA(null).withKD(null).withKP(null).withKS(null).withKV(null).build();

  public static final Constraints flywheelConstraints = Constraints.builder().build();

  public static final GenericFlywheelConstants FLYWHEEL_CONSTANTS =
      GenericFlywheelConstants.builder()
          .withLeaderCANID(-1)
          .withLeaderInversion(InvertedValue.CounterClockwise_Positive)
          .withCanBus(new CANBus())
          .withEnableFOC(false)
          .withCurrentLimit(currentLimits)
          .withMomentOfInertia(0.0)
          .withGearRatio(0.0)
          .withMotorConfig(DCMotor.getKrakenX60(1))
          .withGains(flywheelGains)
          .withConstraints(flywheelConstraints)
          .withAlignedFollowerCANIDs(Set.of())
          .withOpposedFollowerCANIDs(Set.of())
          .build();

  public static final HoodConstants HOOD_CONSTANTS = HoodConstants.builder().build();
}
