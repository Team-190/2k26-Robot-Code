package edu.wpi.team190.gompeilib.core.utility;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.FieldSource;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class GeometryUtilTest {
  public static Translation2d[] TRANSLATION2D_CASES;
  public static Pair<Double, Double>[] XY_PAIR_CASES;
  public static Rotation2d[] ROTATION2D_CASES;
  public static Pose2d[] POSE2D_CASES;

  static {
    TRANSLATION2D_CASES =
        new Translation2d[] {
          Translation2d.kZero, new Translation2d(190.0, 190.0), new Translation2d(42.0, 42.0)
        };
    XY_PAIR_CASES =
        new Pair[] {new Pair<>(0.0, 0.0), new Pair<>(190.0, 190.0), new Pair<>(42.0, 42.0)};
    ROTATION2D_CASES =
        new Rotation2d[] {
          Rotation2d.kZero, Rotation2d.fromDegrees(190.0), Rotation2d.fromDegrees(42.0)
        };
    POSE2D_CASES =
        new Pose2d[] {
          Pose2d.kZero,
          new Pose2d(190.0, 190.0, Rotation2d.fromDegrees(190.0)),
          new Pose2d(42.0, 42.0, Rotation2d.fromDegrees(42.0))
        };
  }

  @Test
  @Order(1)
  void testConstruction() {
    new GeometryUtil();
  }

  @ParameterizedTest
  @FieldSource("TRANSLATION2D_CASES")
  @Order(2)
  void testToTransform2dGivenTranslation(Translation2d translation) {
    assertEquals(
        GeometryUtil.toTransform2d(translation),
        new Transform2d(translation.getX(), translation.getY(), new Rotation2d()));
  }

  @ParameterizedTest
  @FieldSource("XY_PAIR_CASES")
  @Order(3)
  void testToTransform2dGivenDoubles(Pair<Double, Double> xyPair) {
    double x = xyPair.getFirst();
    double y = xyPair.getSecond();

    assertEquals(GeometryUtil.toTransform2d(x, y), new Transform2d(x, y, new Rotation2d()));
  }

  @ParameterizedTest
  @FieldSource("ROTATION2D_CASES")
  @Order(4)
  void testToTransform2dGivenRotation2d(Rotation2d rotation) {
    assertEquals(
        GeometryUtil.toTransform2d(rotation), new Transform2d(new Translation2d(), rotation));
  }

  @ParameterizedTest
  @FieldSource("POSE2D_CASES")
  @Order(5)
  void testToTransform2dGivenPose2d(Pose2d pose) {
    assertEquals(
        GeometryUtil.toTransform2d(pose),
        new Transform2d(pose.getX(), pose.getY(), pose.getRotation()));
  }

  @Test
  @Order(6)
  void testSinglePose2dsAreZero() {
    Pose2d zeroedPose2d = Pose2d.kZero;
    Pose2d nonZeroedPose2d1 = new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0.0));
    Pose2d nonZeroedPose2d2 = new Pose2d(0.0, 1.0, Rotation2d.fromDegrees(0.0));
    Pose2d nonZeroedPose2d3 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(1.0));

    assertTrue(GeometryUtil.isZero(zeroedPose2d));
    assertFalse(GeometryUtil.isZero(nonZeroedPose2d1));
    assertFalse(GeometryUtil.isZero(nonZeroedPose2d2));
    assertFalse(GeometryUtil.isZero(nonZeroedPose2d3));
  }

  @Test
  @Order(7)
  void testAnyPose2dsAreZero() {
    Pose2d[] cases = new Pose2d[] {POSE2D_CASES[0], POSE2D_CASES[1], POSE2D_CASES[2]};
    assertTrue(GeometryUtil.isZero(cases));
  }

  @Test
  @Order(8)
  void testAnyPose2dsAreNotZero() {
    Pose2d[] cases = new Pose2d[] {POSE2D_CASES[1], POSE2D_CASES[2]};
    assertFalse(GeometryUtil.isZero(cases));
  }

  @Test
  @Order(9)
  void testSingleTranslation2dsAreZero() {
    Translation2d zeroedTranslation2d = Translation2d.kZero;
    Translation2d nonZeroedTranslation2d1 = new Translation2d(1.0, 0.0);
    Translation2d nonZeroedTranslation2d2 = new Translation2d(0.0, 1.0);

    assertTrue(GeometryUtil.isZero(zeroedTranslation2d));
    assertFalse(GeometryUtil.isZero(nonZeroedTranslation2d1));
    assertFalse(GeometryUtil.isZero(nonZeroedTranslation2d2));
  }

  @Test
  @Order(10)
  void testSingleRotation2dsAreZero() {
    Rotation2d zeroedRotation2d = Rotation2d.kZero;
    Rotation2d nonZeroedTranslation2d1 = Rotation2d.fromDegrees(190.0);
    Rotation2d nonZeroedTranslation2d2 = Rotation2d.fromDegrees(42.0);

    assertTrue(GeometryUtil.isZero(zeroedRotation2d));
    assertFalse(GeometryUtil.isZero(nonZeroedTranslation2d1));
    assertFalse(GeometryUtil.isZero(nonZeroedTranslation2d2));
  }

  @Test
  @Order(11)
  void testSinglePose2dsAreNAN() {
    Pose2d zeroedPose2d = Pose2d.kZero;
    Pose2d nanPose2d1 = new Pose2d(Double.NaN, 0.0, Rotation2d.fromDegrees(0.0));
    Pose2d nanPose2d2 = new Pose2d(0.0, Double.NaN, Rotation2d.fromDegrees(0.0));
    Pose2d nanPose2d3 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(Double.NaN));

    assertFalse(GeometryUtil.isNaN(zeroedPose2d));
    assertTrue(GeometryUtil.isNaN(nanPose2d1));
    assertTrue(GeometryUtil.isNaN(nanPose2d2));
    assertTrue(GeometryUtil.isNaN(nanPose2d3));
  }

  @Test
  @Order(12)
  void testAnyPose2dsAreNAN() {
    Pose2d nanPose = new Pose2d(Double.NaN, 190.0, Rotation2d.fromDegrees(190.0));
    Pose2d[] cases = new Pose2d[] {nanPose, POSE2D_CASES[0], POSE2D_CASES[1], POSE2D_CASES[2]};
    assertTrue(GeometryUtil.isNaN(cases));
  }

  @Test
  @Order(13)
  void testAnyPose2dsAreNotNAN() {
    Pose2d[] cases = new Pose2d[] {POSE2D_CASES[0], POSE2D_CASES[1], POSE2D_CASES[2]};
    assertFalse(GeometryUtil.isNaN(cases));
  }

  @Test
  @Order(14)
  void testRectangleDoesNotContainsPose() {
    Rectangle2d[] rectangles = {
      new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 1, 1),
      new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 1, 1),
      new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 2, 2)
    };
    Pose2d pose = new Pose2d(-5, -5, Rotation2d.kZero);
    assertFalse(GeometryUtil.contains(rectangles, pose));
  }

  @Test
  @Order(15)
  void testRectangleContainsPose() {
    Rectangle2d[] rectangles = {
      new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 1, 1),
      new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 2, 2),
      new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 3, 3)
    };
    Pose2d pose = new Pose2d(1.5, 1.5, Rotation2d.kZero);
    assertTrue(GeometryUtil.contains(rectangles, pose));
  }

  @Test
  @Order(16)
  void testRectangleDoesNotContainsTranslation() {
    Rectangle2d[] rectangles = {
      new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 1, 1),
      new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 1, 1),
      new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 2, 2)
    };
    Translation2d translation = new Translation2d(-5, -5);
    assertFalse(GeometryUtil.contains(rectangles, translation));
  }

  @Test
  @Order(17)
  void testRectangleContainsTranslation() {
    Rectangle2d[] rectangles = {
      new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 1, 1),
      new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 2, 2),
      new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 3, 3)
    };
    Translation2d translation = new Translation2d(1.5, 1.5);
    assertTrue(GeometryUtil.contains(rectangles, translation));
  }

  @Test
  @Order(18)
  void testRectanglePose2ds() {
    Rectangle2d rectangle = new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 1, 1);
    Pose2d[] poses = GeometryUtil.rectanglePose2ds(rectangle);
    Pose2d[] correctPoses = {
      new Pose2d(1.0, 1.0, Rotation2d.kZero),
      new Pose2d(-1.0, 1.0, Rotation2d.kZero),
      new Pose2d(-1.0, -1.0, Rotation2d.kZero),
      new Pose2d(1.0, -1.0, Rotation2d.kZero),
      new Pose2d(0, 0, Rotation2d.kZero),
    };
    for (int i = 0; i < 5; i++) {
      assertEquals(poses[i], correctPoses[i]);
    }
  }
}
