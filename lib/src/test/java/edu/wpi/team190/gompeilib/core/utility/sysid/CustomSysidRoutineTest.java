package edu.wpi.team190.gompeilib.core.utility.sysid;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

public class CustomSysidRoutineTest {
  @Test
  public void testCustomSysIdRoutine() {
    Subsystem mockSubsystem =
        new Subsystem() {
          @Override
          public void periodic() {}
        };

    CustomSysIdRoutine.Config<VoltageUnit> config =
        new CustomSysIdRoutine.Config<>(
            CustomUnits.voltsPerSecond.ofNative(1.0),
            Volts.of(1.0),
            Seconds.of(5.0),
            state -> {},
            Volts);

    AtomicReference<Measure<VoltageUnit>> drivenValue = new AtomicReference<>();
    CustomSysIdRoutine.Mechanism<VoltageUnit> mechanism =
        new CustomSysIdRoutine.Mechanism<>(drivenValue::set, log -> {}, mockSubsystem, "MockMech");

    CustomSysIdRoutine.Mechanism<VoltageUnit> mechanism2 =
        new CustomSysIdRoutine.Mechanism<>(drivenValue::set, mockSubsystem);

    CustomSysIdRoutine<VoltageUnit> routine =
        new CustomSysIdRoutine<>(config, mechanism, Volts.mutable(0));

    Command quasistaticFwd = routine.quasistatic(CustomSysIdRoutine.Direction.kForward);
    Command quasistaticRev = routine.quasistatic(CustomSysIdRoutine.Direction.kReverse);
    Command dynamicFwd = routine.dynamic(CustomSysIdRoutine.Direction.kForward);
    Command dynamicRev = routine.dynamic(CustomSysIdRoutine.Direction.kReverse);

    assertNotNull(quasistaticFwd);
    assertNotNull(quasistaticRev);
    assertNotNull(dynamicFwd);
    assertNotNull(dynamicRev);
  }
}
