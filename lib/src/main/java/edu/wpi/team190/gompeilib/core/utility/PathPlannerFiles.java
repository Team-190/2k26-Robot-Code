package edu.wpi.team190.gompeilib.core.utility;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.LocalADStar.GridPosition;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PPLibTelemetry;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.locks.ReadWriteLock;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.system.Filesystem;

/**
 * Loads PathPlanner files from a configurable directory.
 *
 * <p>PathPlannerLib always reads from {@code deploy/pathplanner}. Projects that keep separate
 * PathPlanner files per robot can point this class at another directory with {@link
 * #setDirectory(File)} and load paths, robot settings, and the navgrid through it instead.
 */
public final class PathPlannerFiles {
  private static File directory;
  private static final Map<String, PathPlannerPath> pathCache = new HashMap<>();

  private PathPlannerFiles() {}

  /**
   * Sets the directory PathPlanner files are loaded from. Must be called before any files are
   * loaded, including before any {@code SwerveDrive} is constructed.
   *
   * @param pathplannerDirectory directory containing {@code paths/}, {@code settings.json}, and
   *     {@code navgrid.json}
   */
  public static void setDirectory(File pathplannerDirectory) {
    directory = pathplannerDirectory;
    pathCache.clear();
  }

  /**
   * @return the directory PathPlanner files are loaded from, defaulting to {@code
   *     deploy/pathplanner}
   */
  public static File getDirectory() {
    if (directory == null) {
      directory = new File(Filesystem.getDeployDirectory(), "pathplanner");
    }
    return directory;
  }

  private static JSONObject readJson(File file) throws IOException, ParseException {
    return (JSONObject) new JSONParser().parse(Files.readString(file.toPath()));
  }

  /**
   * Loads a path from {@code paths/<pathName>.path}. Equivalent to {@link
   * PathPlannerPath#fromPathFile(String)}, but reads from {@link #getDirectory()}.
   *
   * @param pathName the name of the path, without the file extension
   * @return the path
   */
  public static PathPlannerPath loadPath(String pathName)
      throws IOException, ParseException, FileVersionException {
    if (pathCache.containsKey(pathName)) {
      return pathCache.get(pathName);
    }

    JSONObject json = readJson(new File(getDirectory(), "paths/" + pathName + ".path"));
    String version = json.get("version").toString();
    if (!version.split("\\.")[0].equals("2025")) {
      throw new FileVersionException(version, "2025.X", pathName + ".path");
    }

    PathPlannerPath path;
    try {
      // PathPlannerLib only parses path JSON from its fixed directory, so call its parser directly
      Method fromJson = PathPlannerPath.class.getDeclaredMethod("fromJson", JSONObject.class);
      fromJson.setAccessible(true);
      path = (PathPlannerPath) fromJson.invoke(null, json);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException("Unable to parse path " + pathName, e);
    }
    path.name = pathName;
    PPLibTelemetry.registerHotReloadPath(pathName, path);
    pathCache.put(pathName, path);
    return path;
  }

  /**
   * Loads the robot config from {@code settings.json}. Equivalent to {@link
   * RobotConfig#fromGUISettings()}, but reads from {@link #getDirectory()}.
   *
   * @return the robot config
   */
  public static RobotConfig loadRobotConfig() throws IOException, ParseException {
    JSONObject json = readJson(new File(getDirectory(), "settings.json"));

    boolean isHolonomic = (boolean) json.get("holonomicMode");
    double massKG = ((Number) json.get("robotMass")).doubleValue();
    double MOI = ((Number) json.get("robotMOI")).doubleValue();
    double wheelRadius = ((Number) json.get("driveWheelRadius")).doubleValue();
    double gearing = ((Number) json.get("driveGearing")).doubleValue();
    double maxDriveSpeed = ((Number) json.get("maxDriveSpeed")).doubleValue();
    double wheelCOF = ((Number) json.get("wheelCOF")).doubleValue();
    String driveMotor = (String) json.get("driveMotorType");
    double driveCurrentLimit = ((Number) json.get("driveCurrentLimit")).doubleValue();

    int numMotors = isHolonomic ? 1 : 2;
    DCMotor gearbox =
        switch (driveMotor) {
          case "krakenX60" -> DCMotor.getKrakenX60(numMotors);
          case "krakenX60FOC" -> DCMotor.getKrakenX60Foc(numMotors);
          case "falcon500" -> DCMotor.getFalcon500(numMotors);
          case "falcon500FOC" -> DCMotor.getFalcon500Foc(numMotors);
          case "vortex" -> DCMotor.getNeoVortex(numMotors);
          case "NEO" -> DCMotor.getNEO(numMotors);
          case "CIM" -> DCMotor.getCIM(numMotors);
          case "miniCIM" -> DCMotor.getMiniCIM(numMotors);
          default -> throw new IllegalArgumentException("Invalid motor type: " + driveMotor);
        };
    gearbox = gearbox.withReduction(gearing);

    ModuleConfig moduleConfig =
        new ModuleConfig(
            wheelRadius, maxDriveSpeed, wheelCOF, gearbox, driveCurrentLimit, numMotors);

    if (isHolonomic) {
      Translation2d[] moduleOffsets =
          new Translation2d[] {
            new Translation2d(
                ((Number) json.get("flModuleX")).doubleValue(),
                ((Number) json.get("flModuleY")).doubleValue()),
            new Translation2d(
                ((Number) json.get("frModuleX")).doubleValue(),
                ((Number) json.get("frModuleY")).doubleValue()),
            new Translation2d(
                ((Number) json.get("blModuleX")).doubleValue(),
                ((Number) json.get("blModuleY")).doubleValue()),
            new Translation2d(
                ((Number) json.get("brModuleX")).doubleValue(),
                ((Number) json.get("brModuleY")).doubleValue())
          };
      return new RobotConfig(massKG, MOI, moduleConfig, moduleOffsets);
    } else {
      double trackwidth = ((Number) json.get("robotTrackwidth")).doubleValue();
      return new RobotConfig(massKG, MOI, moduleConfig, trackwidth);
    }
  }

  /**
   * Replaces a pathfinder's navgrid with {@code navgrid.json} from {@link #getDirectory()}. {@link
   * LocalADStar} only loads its navgrid from {@code deploy/pathplanner} when constructed, so the
   * grid is swapped in afterwards and the planner is told to restart.
   *
   * @param adStar the pathfinder to update
   */
  @SuppressWarnings("unchecked")
  public static void loadNavGrid(LocalADStar adStar) throws IOException, ParseException {
    JSONObject json = readJson(new File(getDirectory(), "navgrid.json"));
    double nodeSize = ((Number) json.get("nodeSizeMeters")).doubleValue();
    JSONArray grid = (JSONArray) json.get("grid");
    JSONObject fieldSize = (JSONObject) json.get("field_size");

    try {
      // The planning thread copies its request state under requestLock, so writing the grid under
      // the write lock and requesting a reset makes its next pass plan on the new grid.
      ReadWriteLock requestLock = (ReadWriteLock) field("requestLock").get(adStar);
      requestLock.writeLock().lock();
      try {
        Set<GridPosition> staticObstacles =
            (Set<GridPosition>) field("staticObstacles").get(adStar);
        Set<GridPosition> dynamicObstacles =
            (Set<GridPosition>) field("dynamicObstacles").get(adStar);
        Set<GridPosition> requestObstacles =
            (Set<GridPosition>) field("requestObstacles").get(adStar);
        staticObstacles.clear();
        int nodesX = 0;
        for (int row = 0; row < grid.size(); row++) {
          JSONArray rowArray = (JSONArray) grid.get(row);
          if (row == 0) {
            nodesX = rowArray.size();
          }
          for (int col = 0; col < rowArray.size(); col++) {
            if ((boolean) rowArray.get(col)) {
              staticObstacles.add(new GridPosition(col, row));
            }
          }
        }

        field("nodeSize").setDouble(adStar, nodeSize);
        field("nodesX").setInt(adStar, nodesX);
        field("nodesY").setInt(adStar, grid.size());
        field("fieldLength").setDouble(adStar, ((Number) fieldSize.get("x")).doubleValue());
        field("fieldWidth").setDouble(adStar, ((Number) fieldSize.get("y")).doubleValue());

        requestObstacles.clear();
        requestObstacles.addAll(staticObstacles);
        requestObstacles.addAll(dynamicObstacles);
        field("requestReset").setBoolean(adStar, true);
        field("requestMajor").setBoolean(adStar, true);
        field("requestMinor").setBoolean(adStar, true);
      } finally {
        requestLock.writeLock().unlock();
      }
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException("Unable to load navgrid into LocalADStar", e);
    }
  }

  private static Field field(String name) throws NoSuchFieldException {
    Field field = LocalADStar.class.getDeclaredField(name);
    field.setAccessible(true);
    return field;
  }
}
