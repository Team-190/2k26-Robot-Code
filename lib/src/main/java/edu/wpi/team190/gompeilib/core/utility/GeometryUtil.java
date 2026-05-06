package edu.wpi.team190.gompeilib.core.utility;

import edu.wpi.first.math.geometry.*;

public class GeometryUtil {
  /**
   * Creates a pure translating transform
   *
   * @param translation The translation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure translating transform
   *
   * @param x The x coordinate of the translation
   * @param y The y coordinate of the translation
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(double x, double y) {
    return new Transform2d(x, y, new Rotation2d());
  }

  /**
   * Creates a pure rotating transform
   *
   * @param rotation The rotation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Rotation2d rotation) {
    return new Transform2d(new Translation2d(), rotation);
  }

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Checks if a pose is (0, 0, 0)
   *
   * @param pose The pose to check
   * @return Whether the pose is zero
   */
  public static boolean isZero(Pose2d pose) {
    return pose.getX() == 0.0 && pose.getY() == 0.0 && pose.getRotation().getDegrees() == 0.0;
  }

  /**
   * Checks if any poses are (0, 0, 0)
   *
   * @param poses The poses to check
   * @return Whether any poses are zero
   */
  public static boolean isZero(Pose2d[] poses) {
    for (Pose2d p : poses) {
      if (isZero(p)) {
        return true;
      }
    }
    return false;
  }

  /**
   * Checks if a translation is (0, 0)
   *
   * @param translation The translation to check
   * @return Whether the translation is zero
   */
  public static boolean isZero(Translation2d translation) {
    return translation.getX() == 0.0 && translation.getY() == 0.0;
  }

  /**
   * Checks if a rotation is zero
   *
   * @param rotation The rotation to check
   * @return Whether the rotation is zero
   */
  public static boolean isZero(Rotation2d rotation) {
    return rotation.getDegrees() == 0.0;
  }

  /**
   * Checks if a pose is NaN
   *
   * @param pose The pose to check
   * @return Whether the pose is NaN
   */
  public static boolean isNaN(Pose2d pose) {
    return Double.isNaN(pose.getX())
        || Double.isNaN(pose.getY())
        || Double.isNaN(pose.getRotation().getDegrees());
  }

  /**
   * Checks if any poses are NaN
   *
   * @param poses The poses to check
   * @return Whether any poses are NaN
   */
  public static boolean isNaN(Pose2d[] poses) {
    for (Pose2d p : poses) {
      if (isNaN(p)) {
        return true;
      }
    }
    return false;
  }

  /**
   * @param rectangle2ds Array of rectangles to check if the pose is in
   * @param pose The pose to check is contained by the rectangle
   * @return Whether the pose is contained by the rectangle
   */
  public static boolean contains(Rectangle2d[] rectangle2ds, Pose2d pose) {
    for (Rectangle2d rectangle : rectangle2ds) {
      if (rectangle.contains(pose.getTranslation())) {
        return true;
      }
    }
    return false;
  }

  /**
   * @param rectangle2ds Array of rectangles to check against
   * @param pose Center of the target rectangle
   * @param x Full width of the target rectangle
   * @param y Full height of the target rectangle
   * @return Whether ANY part of the target rectangle is inside any rectangle
   */
  public static boolean intersects(Rectangle2d[] rectangle2ds, Pose2d pose, double x, double y) {
    Rectangle2d target = new Rectangle2d(pose, x, y);

    double hx = x / 2.0;
    double hy = y / 2.0;

    // Target corners (correctly rotated)
    Translation2d[] corners =
        new Translation2d[] {
          new Translation2d(hx, hy),
          new Translation2d(-hx, hy),
          new Translation2d(-hx, -hy),
          new Translation2d(hx, -hy)
        };

    Translation2d center = pose.getTranslation();
    Rotation2d rot = pose.getRotation();

    for (Rectangle2d rectangle : rectangle2ds) {

      for (Translation2d corner : corners) {
        Translation2d worldCorner = center.plus(corner.rotateBy(rot));
        if (rectangle.contains(worldCorner)) {
          return true;
        }
      }

      if (rectangle.contains(center)) {
        return true;
      }

      if (target.contains(rectangle.getCenter().getTranslation())) {
        return true;
      }
    }

    return false;
  }

  /**
   * @param rectangle2ds Array of rectangles to check if the translation is in
   * @param translation2d The translation to check is contained by the rectangle
   * @return Whether the translation is contained by the rectangle
   */
  public static boolean contains(Rectangle2d[] rectangle2ds, Translation2d translation2d) {
    for (Rectangle2d rectangle : rectangle2ds) {
      if (rectangle.contains(translation2d)) {
        return true;
      }
    }
    return false;
  }

  /**
   * Gets the center and corners of a rectangle2d
   *
   * @param rectangle2d The rectangle2d to get the center and corners of
   * @return The poses from the rectangle2d
   */
  public static Pose2d[] rectanglePose2ds(Rectangle2d rectangle2d) {
    Pose2d[] poses = new Pose2d[5];
    poses[0] =
        rectangle2d
            .getCenter()
            .transformBy(
                new Transform2d(
                    rectangle2d.getXWidth(), rectangle2d.getYWidth(), new Rotation2d()));
    poses[1] =
        rectangle2d
            .getCenter()
            .transformBy(
                new Transform2d(
                    -rectangle2d.getXWidth(), rectangle2d.getYWidth(), new Rotation2d()));
    poses[2] =
        rectangle2d
            .getCenter()
            .transformBy(
                new Transform2d(
                    -rectangle2d.getXWidth(), -rectangle2d.getYWidth(), new Rotation2d()));
    poses[3] =
        rectangle2d
            .getCenter()
            .transformBy(
                new Transform2d(
                    rectangle2d.getXWidth(), -rectangle2d.getYWidth(), new Rotation2d()));
    poses[4] = rectangle2d.getCenter();
    return poses;
  }
}
