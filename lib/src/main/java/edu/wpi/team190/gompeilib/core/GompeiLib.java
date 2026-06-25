package edu.wpi.team190.gompeilib.core;

import edu.wpi.team190.gompeilib.core.robot.RobotMode;

/** Main class for the GompeiLib library. Must be initialized by calling init() in robotInit(). */
public final class GompeiLib {

  private static boolean initialized = false;
  private static RobotMode currentMode;
  private static boolean tuningMode;
  private static double loopPeriod;

  /**
   * Initializes the GompeiLib library with global constants from the robot project. This method
   * should be called once in robotInit().
   *
   * @param mode The current robot mode (REAL, SIM, or REPLAY).
   * @param isTuning Whether tuning mode is enabled.
   * @param loopPeriodSecs The robot's main loop period in seconds.
   */
  public static void init(RobotMode mode, boolean isTuning, double loopPeriodSecs) {
    if (initialized) {
      System.err.println("GompeiLib has already been initialized!");
      return;
    }

    currentMode = mode;
    tuningMode = isTuning;
    loopPeriod = loopPeriodSecs;

    initialized = true;
  }

  public static void deinit() {
    if (!initialized) {
      System.err.println("GompeiLib has already been deinitialized!");
      return;
    }

    currentMode = null;
    tuningMode = false;
    loopPeriod = 0.0;

    initialized = false;
  }

  /** Throws an exception if the library has not been initialized. */
  private static void checkInitialized() {
    if (!initialized) {
      throw new IllegalStateException("GompeiLib.init() must be called before using GompeiLib.");
    }
  }

  // Public getters for other classes in your library to use.
  public static RobotMode getMode() {
    checkInitialized();
    return currentMode;
  }

  public static boolean isTuning() {
    checkInitialized();
    return tuningMode;
  }

  public static double getLoopPeriod() {
    checkInitialized();
    return loopPeriod;
  }
}
