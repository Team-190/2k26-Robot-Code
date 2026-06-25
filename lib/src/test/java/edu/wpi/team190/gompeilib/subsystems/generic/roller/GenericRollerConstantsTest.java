package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import java.util.Set;
import org.junit.jupiter.api.Test;

public class GenericRollerConstantsTest {
  @Test
  public void testConstantsAndBuilder() {
    GenericRollerConstants constants =
        GenericRollerConstants.builder()
            .withLeaderCANID(1)
            .withLeaderInvertedValue(InvertedValue.Clockwise_Positive)
            .withAlignedFollowerCANID(2)
            .withOpposedFollowerCANID(3)
            .withCurrentLimits(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(30.0)
                    .withStatorCurrentLimit(30.0)
                    .build())
            .withRollerGearbox(DCMotor.getNeo550(1))
            .withRollerMotorGearRatio(5.0)
            .withMomentOfInertia(Units.KilogramSquareMeters.of(0.001))
            .withNeutralMode(NeutralModeValue.Brake)
            .withCanBus(new CANBus("rio"))
            .withEnableFOC(false)
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .build();

    assertNotNull(constants);
    assertEquals(1, constants.leaderCANID);
    assertEquals(InvertedValue.Clockwise_Positive, constants.leaderInvertedValue);
    assertEquals(Set.of(2), constants.alignedFollowerCANIDs);
    assertEquals(Set.of(3), constants.opposedFollowerCANIDs);
    assertEquals(5.0, constants.rollerMotorGearRatio);
    assertEquals(NeutralModeValue.Brake, constants.neutralMode);
    assertFalse(constants.enableFOC);
  }
}
