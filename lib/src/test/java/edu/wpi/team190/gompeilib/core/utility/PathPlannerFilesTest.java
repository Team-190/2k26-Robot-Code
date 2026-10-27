package edu.wpi.team190.gompeilib.core.utility;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.LocalADStar.GridPosition;
import com.pathplanner.lib.util.FileVersionException;
import java.io.File;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.math.geometry.Translation2d;

public class PathPlannerFilesTest {
  private static final String PATH_JSON =
      """
      {
        "version": "2025.0",
        "waypoints": [
          {
            "anchor": {"x": 1.0, "y": 2.0},
            "prevControl": null,
            "nextControl": {"x": 2.0, "y": 2.0},
            "isLocked": false,
            "linkedName": null
          },
          {
            "anchor": {"x": 5.0, "y": 2.0},
            "prevControl": {"x": 4.0, "y": 2.0},
            "nextControl": null,
            "isLocked": false,
            "linkedName": null
          }
        ],
        "rotationTargets": [],
        "constraintZones": [],
        "pointTowardsZones": [],
        "eventMarkers": [],
        "globalConstraints": {
          "maxVelocity": 3.0,
          "maxAcceleration": 3.0,
          "maxAngularVelocity": 540.0,
          "maxAngularAcceleration": 720.0,
          "nominalVoltage": 12.0,
          "unlimited": false
        },
        "goalEndState": {"velocity": 0.0, "rotation": 0.0},
        "reversed": false,
        "folder": null,
        "idealStartingState": {"velocity": 0.0, "rotation": 0.0},
        "useDefaultConstraints": true
      }
      """;

  private static final String SETTINGS_JSON =
      """
      {
        "holonomicMode": true,
        "robotMass": 67.0,
        "robotMOI": 7.897,
        "robotTrackwidth": 0.546,
        "driveWheelRadius": 0.0508,
        "driveGearing": 7.03,
        "maxDriveSpeed": 4.38,
        "driveMotorType": "krakenX60FOC",
        "driveCurrentLimit": 60.0,
        "wheelCOF": 2.0,
        "flModuleX": 0.251,
        "flModuleY": 0.308,
        "frModuleX": 0.251,
        "frModuleY": -0.308,
        "blModuleX": -0.251,
        "blModuleY": 0.308,
        "brModuleX": -0.251,
        "brModuleY": -0.308
      }
      """;

  // 3 columns x 2 rows, with obstacles at (col 0, row 0) and (col 2, row 1)
  private static final String NAVGRID_JSON =
      """
      {
        "field_size": {"x": 1.5, "y": 1.0},
        "nodeSizeMeters": 0.5,
        "grid": [[true, false, false], [false, false, true]]
      }
      """;

  @TempDir Path tempDir;

  @BeforeAll
  static void setup() {
    org.wpilib.hardware.hal.HAL.initialize();
  }

  @BeforeEach
  void writeFiles() throws Exception {
    Files.createDirectories(tempDir.resolve("paths"));
    Files.writeString(tempDir.resolve("paths/TEST_PATH.path"), PATH_JSON);
    Files.writeString(
        tempDir.resolve("paths/OLD_PATH.path"), PATH_JSON.replace("2025.0", "2024.0"));
    Files.writeString(tempDir.resolve("settings.json"), SETTINGS_JSON);
    Files.writeString(tempDir.resolve("navgrid.json"), NAVGRID_JSON);
    PathPlannerFiles.setDirectory(tempDir.toFile());
  }

  @AfterEach
  void resetDirectory() {
    PathPlannerFiles.setDirectory(null);
  }

  @Test
  void testDefaultDirectoryIsDeployPathplanner() {
    PathPlannerFiles.setDirectory(null);
    assertEquals("pathplanner", PathPlannerFiles.getDirectory().getName());
  }

  @Test
  void testLoadPathFromDirectory() throws Exception {
    PathPlannerPath path = PathPlannerFiles.loadPath("TEST_PATH");

    assertEquals("TEST_PATH", path.name);
    assertEquals(2, path.getWaypoints().size());
    assertEquals(new Translation2d(1.0, 2.0), path.getWaypoints().get(0).anchor());
    assertEquals(new Translation2d(5.0, 2.0), path.getWaypoints().get(1).anchor());
    assertSame(path, PathPlannerFiles.loadPath("TEST_PATH"), "paths should be cached");
  }

  @Test
  void testLoadPathRejectsOldVersion() {
    assertThrows(FileVersionException.class, () -> PathPlannerFiles.loadPath("OLD_PATH"));
  }

  @Test
  void testSetDirectoryClearsPathCache() throws Exception {
    PathPlannerPath first = PathPlannerFiles.loadPath("TEST_PATH");
    PathPlannerFiles.setDirectory(tempDir.toFile());
    assertNotSame(first, PathPlannerFiles.loadPath("TEST_PATH"));
  }

  @Test
  void testLoadRobotConfig() throws Exception {
    RobotConfig config = PathPlannerFiles.loadRobotConfig();

    assertTrue(config.isHolonomic);
    assertEquals(67.0, config.massKG);
    assertEquals(7.897, config.MOI);
    assertEquals(0.0508, config.moduleConfig.wheelRadiusMeters);
    assertEquals(4.38, config.moduleConfig.maxDriveVelocityMPS);
    assertArrayEquals(
        new Translation2d[] {
          new Translation2d(0.251, 0.308),
          new Translation2d(0.251, -0.308),
          new Translation2d(-0.251, 0.308),
          new Translation2d(-0.251, -0.308)
        },
        config.moduleLocations);
  }

  @Test
  @SuppressWarnings("unchecked")
  void testLoadNavGrid() throws Exception {
    LocalADStar adStar = new LocalADStar();
    PathPlannerFiles.loadNavGrid(adStar);

    assertEquals(0.5, get(adStar, "nodeSize"));
    assertEquals(3, get(adStar, "nodesX"));
    assertEquals(2, get(adStar, "nodesY"));
    assertEquals(1.5, get(adStar, "fieldLength"));
    assertEquals(1.0, get(adStar, "fieldWidth"));
    Set<GridPosition> expected = Set.of(new GridPosition(0, 0), new GridPosition(2, 1));
    assertEquals(expected, get(adStar, "staticObstacles"));
    assertEquals(expected, get(adStar, "requestObstacles"));
  }

  @Test
  void testMissingFileThrows() {
    PathPlannerFiles.setDirectory(new File(tempDir.toFile(), "missing"));
    assertThrows(Exception.class, PathPlannerFiles::loadRobotConfig);
    assertThrows(Exception.class, () -> PathPlannerFiles.loadPath("TEST_PATH"));
  }

  private static Object get(LocalADStar adStar, String name) throws Exception {
    Field field = LocalADStar.class.getDeclaredField(name);
    field.setAccessible(true);
    return field.get(adStar);
  }
}
