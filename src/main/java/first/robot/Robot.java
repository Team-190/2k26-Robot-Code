package first.robot;

import static org.wpilib.units.Units.Volts;

import choreo.Choreo;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.utility.PathPlannerFiles;
import edu.wpi.team190.gompeilib.core.utility.VirtualSubsystem;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;
import first.robot.shared.util.Elastic;
import first.robot.shared.util.HubActivePeriod;
import first.robot.shared.util.LocalADStarAK;
import first.robot.v0_Funky.V0_FunkyRobotContainer;
import first.robot.v1_DoomSpiral.V1_DoomSpiralRobotContainer;
import first.robot.v2_Turnover.V2_TurnoverRobotContainer;
import java.io.File;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.IterativeRobotBase;
import org.wpilib.math.util.MathShared;
import org.wpilib.math.util.MathSharedStore;
import org.wpilib.net.WebServer;
import org.wpilib.system.Filesystem;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.system.Watchdog;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final double LOW_BATTERY_VOLTAGE = 10.0;
  private static final double LOW_BATTERY_DISABLED_TIME = 1.5;

  private static final double LOOP_OVERRUN_WARNING_TIMEOUT = 1;
  private static double startupTimestamp = Double.NEGATIVE_INFINITY;
  private final Timer disabledTimer = new Timer();
  private final Alert logReceiverQueueAlert =
      new Alert("123", "Logging queue exceeded capacity, data will NOT be logged.", Level.HIGH);
  private final Elastic.Notification lowBatteryAlert =
      new Elastic.Notification(
          Elastic.NotificationLevel.WARNING,
          "Low Battery! Replace battery soon!",
          "Battery voltage is very low, consider turning off the robot or replacing the battery.");
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    super(Constants.LOOP_PERIOD_SECONDS);

    // Each robot keeps its own Choreo and PathPlanner files under deploy/<robot>. This must run
    // before anything loads paths, including the pathfinder below.
    String DEPLOY_DIR =
        Filesystem.getDeployDirectory().getPath()
            + "/"
            + RobotConfig.ROBOT.name().toLowerCase().replaceFirst("_sim", "");
    try {
      var m = Choreo.class.getDeclaredMethod("setChoreoDir", File.class);
      m.setAccessible(true);
      m.invoke(null, new File(DEPLOY_DIR + "/choreo"));
    } catch (NoSuchMethodException | InvocationTargetException | IllegalAccessException e) {
      throw new RuntimeException(e);
    }
    // PathPlanner's GUI opens a non-Gradle folder's deploy/pathplanner, hence the nested deploy
    PathPlannerFiles.setDirectory(new File(DEPLOY_DIR + "/deploy/pathplanner"));

    Pathfinding.setPathfinder(new LocalADStarAK());
    SignalLogger.enableAutoLogging(false);
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("GompeiLib_Version", BuildConstants.VERSION);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        // setUseTiming(false);
        Logger.addDataReceiver(new NT4Publisher());
        // setting up maple sim field
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Initialize GompeiLib
    GompeiLib.init(Constants.getMode(), Constants.TUNING_MODE, Constants.LOOP_PERIOD_SECONDS);

    // Start timers
    disabledTimer.restart();

    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(LOOP_OVERRUN_WARNING_TIMEOUT);
    } catch (Exception e) {
      DriverStationErrors.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(LOOP_OVERRUN_WARNING_TIMEOUT);

    // Silence Rotation2d warnings
    var mathShared = MathSharedStore.getMathShared();
    MathSharedStore.setMathShared(
        new MathShared() {
          @Override
          public void reportError(String error, StackTraceElement[] stackTrace) {
            if (error.startsWith("x and y components of Rotation2d are zero")) {
              return;
            }
            mathShared.reportError(error, stackTrace);
          }

          @Override
          public double getTimestamp() {
            return mathShared.getTimestamp();
          }
        });

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    WebServer.start(5800, DEPLOY_DIR);

    RobotController.setBrownoutVoltages(Volts.of(6), Volts.of(7));
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer =
        switch (RobotConfig.ROBOT) {
          case V0_FUNKY, V0_FUNKY_SIM -> new V0_FunkyRobotContainer();
          case V1_DOOMSPIRAL, V1_DOOMSPIRAL_SIM -> new V1_DoomSpiralRobotContainer();
          case V2_TURNOVER, V2_TURNOVER_SIM -> new V2_TurnoverRobotContainer();
          default -> new RobotContainer() {};
        };

    Elastic.selectTab("Autonomous");

    startupTimestamp = Timer.getTimestamp();
  }

  public static boolean isJitting() {
    return Timer.getTimestamp() - startupTimestamp <= 45.0;
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.

    PhoenixUtil.refreshAll();
    VirtualSubsystem.periodicAll();
    robotContainer.robotPeriodic();
    CommandScheduler.getInstance().run();

    // Check logging fault
    logReceiverQueueAlert.set(Logger.getReceiverQueueFault());

    // Update low battery alert
    if (RobotState.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() < LOW_BATTERY_VOLTAGE
        && disabledTimer.hasElapsed(LOW_BATTERY_DISABLED_TIME)) {
      Elastic.sendNotification(lowBatteryAlert);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Elastic.selectTab("Autonomous");

    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    HubActivePeriod.initialize();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(autonomousCommand);
    }
    Elastic.selectTab("Teleoperated");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
