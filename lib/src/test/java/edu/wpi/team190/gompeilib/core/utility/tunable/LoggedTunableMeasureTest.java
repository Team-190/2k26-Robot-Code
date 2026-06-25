package edu.wpi.team190.gompeilib.core.utility.tunable;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.mockito.Mockito.atLeastOnce;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import java.util.Arrays;
import org.junit.jupiter.api.*;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.mockito.MockedConstruction;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class LoggedTunableMeasureTest {
  @BeforeEach
  void setUp() {
    GompeiLib.deinit();
  }

  @Test
  @Order(1)
  public void defaultValueTuningMode() {
    GompeiLib.init(null, true, 0.02);
    LoggedTunableMeasure<DistanceUnit> number =
        new LoggedTunableMeasure<>("number", Meters.of(190.0));
    assertEquals(190.0, number.get(Meters));
  }

  @Test
  @Order(2)
  public void defaultValueNoTuningMode() {
    GompeiLib.init(null, false, 0.02);
    LoggedTunableMeasure<DistanceUnit> number =
        new LoggedTunableMeasure<>("number", Meters.of(190.0));
    assertEquals(190.0, number.get(Meters));
  }

  @Test
  @Order(3)
  public void testGetTuningDisabled() {
    // 1. Initialize GompeiLib with tuning = false
    GompeiLib.init(null, false, 0.02);

    double expectedDefault = 12.34;
    LoggedTunableMeasure<DistanceUnit> tunable =
        new LoggedTunableMeasure<>("TestKey", Meters.of(expectedDefault));

    assertEquals(
        expectedDefault,
        tunable.get(Meters),
        "Should return the default value when tuning is disabled");
  }

  @Test
  @Order(4)
  public void testGetTuningEnabled() {
    // 1. Initialize GompeiLib with tuning = true
    GompeiLib.init(null, true, 0.02);

    double defaultValue = 1.0;
    double dashboardValue = 99.9;

    // 2. Mock the construction of LoggedNetworkNumber
    try (MockedConstruction<LoggedNetworkNumber> mocked =
        mockConstruction(
            LoggedNetworkNumber.class,
            (mock, context) -> {
              // When dashboardNumber.get() is called, return our fake dashboard value
              when(mock.get()).thenReturn(dashboardValue);
            })) {

      // 3. Initialize the tunable (this triggers 'new LoggedNetworkNumber')
      LoggedTunableMeasure<DistanceUnit> tunable =
          new LoggedTunableMeasure<>("TestKey", Meters.of(defaultValue));

      // 4. Verify that it returns the dashboard value, NOT the default
      assertEquals(
          dashboardValue,
          tunable.get(Meters),
          "Should return the dashboard value when tuning is enabled");

      // 5. Verify the constructor was actually called with the right params
      LoggedNetworkNumber mockInternal = mocked.constructed().get(0);
      verify(mockInternal, atLeastOnce()).get();
    }
  }

  @Test
  @Order(5)
  public void testHasChangedFirstCall() {
    GompeiLib.init(null, false, 0.02);
    LoggedTunableMeasure<DistanceUnit> tunable = new LoggedTunableMeasure<>("Test", Meters.of(5.0));

    // lastValue is null in the map for ID 1
    assertTrue(tunable.hasChanged(1), "First call for a new ID should always return true");
  }

  @Test
  @Order(6)
  public void testHasChangedNoUpdate() {
    GompeiLib.init(null, false, 0.02);
    LoggedTunableMeasure<DistanceUnit> tunable = new LoggedTunableMeasure<>("Test", Meters.of(5.0));

    tunable.hasChanged(1); // First call sets the map entry for ID 1 to 5.0

    // Second call: currentValue (5.0) == lastValue (5.0)
    assertFalse(
        tunable.hasChanged(1), "Should return false if the value hasn't changed since last check");
  }

  @Test
  @Order(7)
  public void testHasChangedWithNewValue() {
    GompeiLib.init(null, true, 0.02);

    // We use a Mock to control what get() returns manually
    try (MockedConstruction<LoggedNetworkNumber> mocked =
        mockConstruction(LoggedNetworkNumber.class)) {
      LoggedTunableMeasure<DistanceUnit> tunable =
          new LoggedTunableMeasure<>("Test", Meters.of(1.0));
      LoggedNetworkNumber mockNT = mocked.constructed().get(0);

      // Setup initial state
      when(mockNT.get()).thenReturn(1.0);
      tunable.hasChanged(1);

      // Change the value
      when(mockNT.get()).thenReturn(2.0);

      assertTrue(tunable.hasChanged(1), "Should return true when the dashboard value changes");
      assertFalse(tunable.hasChanged(1), "Should return false again if called immediately after");
    }
  }

  @Test
  @Order(8)
  public void testHasChangedIdIsolation() {
    GompeiLib.init(null, false, 0.02);
    LoggedTunableMeasure<DistanceUnit> tunable =
        new LoggedTunableMeasure<>("Test", Meters.of(10.0));

    // ID 1 checks the value
    assertTrue(tunable.hasChanged(1));
    assertFalse(tunable.hasChanged(1)); // ID 1 is now "up to date"

    // ID 2 checks for the first time
    assertTrue(tunable.hasChanged(2), "ID 2 should return true even if ID 1 already checked");
  }

  @Test
  @Order(9)
  public void testIfChangedConsumerTriggers() {
    GompeiLib.init(null, false, 0.02);
    LoggedTunableMeasure<DistanceUnit> num1 = new LoggedTunableMeasure<>("n1", Meters.of(1.0));
    LoggedTunableMeasure<DistanceUnit> num2 = new LoggedTunableMeasure<>("n2", Meters.of(2.0));
    int id = 777;

    // We use an array to "capture" the results from inside the lambda
    final double[][] capturedValues = new double[1][];

    // First call: Should trigger because they are new to this ID
    LoggedTunableMeasure.ifChanged(
        id,
        (values) ->
            capturedValues[0] =
                Arrays.stream(values).mapToDouble((value) -> value.in(Meters)).toArray(),
        num1,
        num2);

    assertNotNull(capturedValues[0], "Action should have been called");
    assertEquals(1.0, capturedValues[0][0]);
    assertEquals(2.0, capturedValues[0][1]);
  }

  @Test
  @Order(10)
  void testIfChangedRunnableDoesNotTrigger() {
    GompeiLib.init(null, false, 0.02);
    LoggedTunableMeasure<DistanceUnit> num1 = new LoggedTunableMeasure<>("n1", Meters.of(1.0));
    int id = 888;

    // Manually consume the first "true"
    num1.hasChanged(id);

    // Track if the runnable runs
    final boolean[] ran = {false};

    // Call ifChanged: Since value hasn't changed since the line above, it shouldn't run
    LoggedTunableMeasure.ifChanged(id, () -> ran[0] = true, num1);

    assertFalse(ran[0], "Action should NOT run if nothing changed");
  }

  @Test
  @Order(11)
  void testIfChangedTriggersIfOnlyOneChanges() {
    GompeiLib.init(null, true, 0.02);
    int id = 999;

    try (MockedConstruction<LoggedNetworkNumber> mocked =
        mockConstruction(LoggedNetworkNumber.class)) {
      LoggedTunableMeasure<DistanceUnit> n1 = new LoggedTunableMeasure<>("n1", Meters.of(1.0));
      LoggedTunableMeasure<DistanceUnit> n2 = new LoggedTunableMeasure<>("n2", Meters.of(2.0));
      LoggedNetworkNumber mockNT1 = mocked.constructed().get(0);
      LoggedNetworkNumber mockNT2 = mocked.constructed().get(1);

      // Set initial state for both
      when(mockNT1.get()).thenReturn(1.0);
      when(mockNT2.get()).thenReturn(2.0);
      n1.hasChanged(id);
      n2.hasChanged(id);

      // Change ONLY n1
      when(mockNT1.get()).thenReturn(5.0);

      final boolean[] ran = {false};
      LoggedTunableMeasure.ifChanged(
          id,
          (values) -> {
            ran[0] = true;
            assertEquals(5.0, values[0].in(Meters));
            assertEquals(2.0, values[1].in(Meters));
          },
          n1,
          n2);

      assertTrue(ran[0], "Action should trigger if even one value changes");
    }
  }

  @Test
  @Order(12)
  void testIfChangedRunnableOverloadCoverage() {
    // 1. Setup in non-tuning mode for simplicity
    GompeiLib.init(null, false, 0.02);
    LoggedTunableMeasure<DistanceUnit> num = new LoggedTunableMeasure<>("Coverage", Meters.of(1.0));
    int id = 10101;

    // We need a counter to verify the Runnable actually ran
    final int[] callCount = {0};
    Runnable mockAction = () -> callCount[0]++;

    // 2. First call: Should trigger (initial change)
    // This executes the line: ifChanged(id, values -> action.run(), tunableNumbers);
    LoggedTunableMeasure.ifChanged(id, mockAction, num);

    // 3. Assertions
    assertEquals(1, callCount[0], "The Runnable action should have executed once");

    // 4. Second call: Should NOT trigger (no change)
    LoggedTunableMeasure.ifChanged(id, mockAction, num);
    assertEquals(1, callCount[0], "The Runnable action should NOT have executed a second time");
  }

  @Test
  @Order(13)
  void testIfChangedSecondElementTriggers() {
    GompeiLib.init(null, true, 0.02);
    int id = 12345;

    try (MockedConstruction<LoggedNetworkNumber> mocked =
        mockConstruction(LoggedNetworkNumber.class)) {

      LoggedTunableMeasure<DistanceUnit> n1 = new LoggedTunableMeasure<>("n1", Meters.of(1.0));
      LoggedTunableMeasure<DistanceUnit> n2 = new LoggedTunableMeasure<>("n2", Meters.of(2.0));

      LoggedNetworkNumber mockNT1 = mocked.constructed().get(0);
      LoggedNetworkNumber mockNT2 = mocked.constructed().get(1);

      // Initial values
      when(mockNT1.get()).thenReturn(1.0);
      when(mockNT2.get()).thenReturn(2.0);
      n1.hasChanged(id);
      n2.hasChanged(id);

      // Only SECOND value changes
      when(mockNT2.get()).thenReturn(5.0);

      final boolean[] ran = {false};

      LoggedTunableMeasure.ifChanged(id, values -> ran[0] = true, n1, n2);

      assertTrue(ran[0], "Action should trigger when the second element changes");
    }
  }

  @Test
  @Order(14)
  void testIfChangedWithNoTunables() {
    GompeiLib.init(null, false, 0.02);
    int id = 4242;

    final boolean[] ran = {false};

    LoggedTunableMeasure.ifChanged(
        id, (values) -> ran[0] = true
        // no tunable numbers
        );

    assertFalse(ran[0], "Action should not run when no tunables are provided");
  }
}
