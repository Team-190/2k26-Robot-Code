package edu.wpi.team190.gompeilib.core.utility.sysid;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.*;

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

    CustomSysIdRoutine.Mechanism<VoltageUnit> _ =
        new CustomSysIdRoutine.Mechanism<>(drivenValue::set, mockSubsystem);

    CustomSysIdRoutine<VoltageUnit> routine = new CustomSysIdRoutine<>(config, mechanism);

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
