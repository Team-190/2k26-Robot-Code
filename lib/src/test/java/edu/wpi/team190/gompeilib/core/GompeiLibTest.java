package edu.wpi.team190.gompeilib.core;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.List;
import lombok.Builder;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.FieldSource;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class GompeiLibTest {

  private static final List<GompeiLibTestParameters> TEST_PARAMETERS =
      List.of(
          GompeiLibTestParameters.builder()
              .withMode(RobotMode.REAL)
              .withIsTuning(true)
              .withLoopPeriodSecs(0.02)
              .build(),
          GompeiLibTestParameters.builder()
              .withMode(RobotMode.REAL)
              .withIsTuning(false)
              .withLoopPeriodSecs(0.02)
              .build(),
          GompeiLibTestParameters.builder()
              .withMode(RobotMode.SIM)
              .withIsTuning(true)
              .withLoopPeriodSecs(0.02)
              .build(),
          GompeiLibTestParameters.builder()
              .withMode(RobotMode.SIM)
              .withIsTuning(false)
              .withLoopPeriodSecs(0.02)
              .build(),
          GompeiLibTestParameters.builder()
              .withMode(RobotMode.REPLAY)
              .withIsTuning(true)
              .withLoopPeriodSecs(0.02)
              .build(),
          GompeiLibTestParameters.builder()
              .withMode(RobotMode.REPLAY)
              .withIsTuning(false)
              .withLoopPeriodSecs(0.02)
              .build());

  @BeforeEach
  void resetGompeiLib() {
    GompeiLib.deinit();
  }

  @Test
  @Order(1)
  void testConstruction() {
    new GompeiLib();
  }

  @Test
  @Order(2)
  void testInitializationWhenNotInitialized() throws Exception {
    Method checkInitializedMethod = GompeiLib.class.getDeclaredMethod("checkInitialized");
    checkInitializedMethod.setAccessible(true);
    InvocationTargetException ex =
        assertThrows(
            java.lang.reflect.InvocationTargetException.class,
            () -> checkInitializedMethod.invoke(null));

    // Make sure the cause is IllegalStateException
    assertInstanceOf(IllegalStateException.class, ex.getCause());
  }

  @Test
  @Order(3)
  void testInitializationWhenAlreadyInitialized() {
    // Initialize once
    GompeiLib.init(RobotMode.SIM, true, 0.02);

    // Capture System.err output
    ByteArrayOutputStream errContent = new ByteArrayOutputStream();
    PrintStream originalErr = System.err;
    System.setErr(new PrintStream(errContent));

    // Call init again -> should hit the branch
    GompeiLib.init(RobotMode.SIM, true, 0.02);

    // Restore System.err
    System.setErr(originalErr);

    // Check that the correct message was printed
    String output = errContent.toString();
    assertTrue(output.contains("GompeiLib has already been initialized!"));
  }

  @Test
  @Order(4)
  void testDeinitializationFlow() {
    GompeiLib.init(RobotMode.SIM, true, 0.02);
    GompeiLib.deinit();

    // The getters should throw IllegalStateException now because we are deinitialized
    assertThrows(IllegalStateException.class, GompeiLib::getMode);
  }

  @Test
  @Order(5)
  void testDoubleDeinitWarning() {
    GompeiLib.deinit(); // Ensure it is already deinitialized

    ByteArrayOutputStream errContent = new ByteArrayOutputStream();
    PrintStream originalErr = System.err;
    System.setErr(new PrintStream(errContent));

    try {
      GompeiLib.deinit(); // Call it while the stream is redirected
    } finally {
      System.setErr(originalErr); // Always restore in a finally block for safety
    }

    assertTrue(errContent.toString().contains("GompeiLib has already been deinitialized!"));
  }

  @ParameterizedTest
  @FieldSource("TEST_PARAMETERS")
  @Order(6)
  void testGompeiLib(GompeiLibTestParameters params) {
    GompeiLib.init(params.mode, params.isTuning, params.loopPeriodSecs);

    assertEquals(GompeiLib.getMode(), params.mode);
    assertEquals(GompeiLib.isTuning(), params.isTuning);
    assertEquals(GompeiLib.getLoopPeriod(), params.loopPeriodSecs);
  }

  @Builder(setterPrefix = "with")
  private record GompeiLibTestParameters(RobotMode mode, boolean isTuning, double loopPeriodSecs) {}
}
