package edu.wpi.team190.gompeilib.core.utility;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.junit.jupiter.api.*;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class SetpointTest {

  @Test
  @Order(1)
  public void testApplyZero() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);

    Setpoint<DistanceUnit> setpoint = new Setpoint<>(d, s);
    assertEquals(d, setpoint.getNewSetpoint());
  }

  @Test
  @Order(2)
  public void testApplyIncrement() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);

    Setpoint<DistanceUnit> setpoint = new Setpoint<>(d, s);
    setpoint.increment();

    assertEquals(d.plus(s), setpoint.getNewSetpoint());
  }

  @Test
  @Order(3)
  public void testApplyDecrement() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);

    Setpoint<DistanceUnit> setpoint = new Setpoint<>(d, s);
    setpoint.decrement();

    assertEquals(d.minus(s), setpoint.getNewSetpoint());
  }

  @Test
  @Order(4)
  public void testApplyOffsetRepeatedly() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);

    Setpoint<DistanceUnit> setpoint = new Setpoint<>(d, s);

    setpoint.decrement();
    setpoint.decrement();
    setpoint.decrement();
    setpoint.decrement();
    setpoint.increment();
    setpoint.decrement();
    setpoint.increment();

    assertEquals(d.minus(s).minus(s).minus(s), setpoint.getNewSetpoint());
  }

  @Test
  @Order(5)
  public void testResetOffset() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);

    Setpoint<DistanceUnit> setpoint = new Setpoint<>(d, s);
    setpoint.decrement();
    setpoint.reset();

    assertEquals(d, setpoint.getNewSetpoint());
  }

  @Test
  @Order(6)
  public void testSpecificIncrement() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Distance a = Inches.of(4);

    Setpoint<DistanceUnit> setpoint = new Setpoint<>(d, s);
    setpoint.increment(a);

    assertEquals(d.plus(a), setpoint.getNewSetpoint());
  }

  @Test
  @Order(7)
  public void testSpecificDecrement() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(0.1);
    Distance a = Inches.of(4);

    Setpoint<DistanceUnit> setpoint = new Setpoint<>(d, s);
    setpoint.decrement(a);

    assertEquals(d.minus(a), setpoint.getNewSetpoint());
  }

  @Test
  @Order(8)
  public void testApplyGhostOffsetMaximum() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);

    Setpoint<DistanceUnit> setpoint = new Setpoint<>(d, s, min, max);

    setpoint.increment();
    setpoint.increment();
    setpoint.increment();

    assertEquals(max, setpoint.getNewSetpoint());
  }

  @Test
  @Order(9)
  public void testApplyGhostOffsetMinimum() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);

    Setpoint<DistanceUnit> setpoint = new Setpoint<>(d, s, min, max);

    setpoint.decrement();
    setpoint.decrement();
    setpoint.decrement();

    assertEquals(min, setpoint.getNewSetpoint());
  }

  @Test
  @Order(10)
  public void testApplyGhostOffsetMaximumOverride() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);

    Setpoint<DistanceUnit> setpoint = new Setpoint<>(d, s, min, max);

    setpoint.increment(s);
    setpoint.increment(s);
    setpoint.increment(s);

    assertEquals(max, setpoint.getNewSetpoint());
  }

  @Test
  @Order(11)
  public void testApplyGhostOffsetMinimumOverride() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);

    Setpoint<DistanceUnit> setpoint = new Setpoint<>(d, s, min, max);

    setpoint.decrement(s);
    setpoint.decrement(s);
    setpoint.decrement(s);

    assertEquals(min, setpoint.getNewSetpoint());
  }

  @Test
  @Order(12)
  public void testOffsetGetter() {
    Distance d = Meters.of(1);
    Distance s = Meters.of(1);
    Distance min = Meters.of(0);
    Distance max = Meters.of(2);

    Setpoint<DistanceUnit> setpoint = new Setpoint<>(d, s, min, max);
    setpoint.increment(s);

    assertEquals(Meters.of(1.0), setpoint.getOffset());
  }

  @Test
  @Order(13)
  public void testGhostOffsetMax() {
    Voltage v = Volts.of(11);
    Voltage s = Volts.of(1);
    Voltage min = Volts.of(-12);
    Voltage max = Volts.of(12);

    Setpoint<VoltageUnit> setpoint = new Setpoint<>(v, s, min, max);

    setpoint.increment();
    assertEquals(Volts.of(1.0), setpoint.getOffset());

    setpoint.increment();

    Voltage a = (Voltage) setpoint.getNewSetpoint();
    assertEquals(Volts.of(12.0), a);
  }
}
