package edu.wpi.team190.gompeilib.subsystems.vision.camera;

import edu.wpi.first.math.util.Units;

public enum CameraType {
  LIMELIGHT_2_PLUS(
      Limelight2PlusConstants.HORIZONTAL_FOV,
      Limelight2PlusConstants.VERTICAL_FOV,
      Limelight2PlusConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight2PlusConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight2PlusConstants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT),
  LIMELIGHT_3(
      Limelight3Constants.HORIZONTAL_FOV,
      Limelight3Constants.VERTICAL_FOV,
      Limelight3Constants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight3Constants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight3Constants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT),
  LIMELIGHT_3G(
      Limelight3GConstants.HORIZONTAL_FOV,
      Limelight3GConstants.VERTICAL_FOV,
      Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight3GConstants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT),
  LIMELIGHT_4(
      Limelight4Constants.HORIZONTAL_FOV,
      Limelight4Constants.VERTICAL_FOV,
      Limelight4Constants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight4Constants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight4Constants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT),
  THRIFTYCAM(
      ThriftyCamConstants.HORIZONTAL_FOV,
      ThriftyCamConstants.VERTICAL_FOV,
      ThriftyCamConstants.SINGLETAG_XY_STANDARD_DEVIATION_COEFFICIENT,
      ThriftyCamConstants.MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT,
      ThriftyCamConstants.THETA_STANDARD_DEVIATION_COEFFICIENT),
  DEFAULT();

  public final double horizontalFOV;
  public final double verticalFOV;
  public final double primaryXYStandardDeviationCoefficient;
  public final double secondaryXYStandardDeviationCoefficient;
  public final double primaryThetaStandardDeviationCoefficient;

  private CameraType(
      double horizontalFOV,
      double verticalFOV,
      double primaryXYStandardDeviationCoefficient,
      double secondaryXYStandardDeviationCoefficient,
      double primaryThetaStandardDeviationCoefficient) {
    this.horizontalFOV = horizontalFOV;
    this.verticalFOV = verticalFOV;
    this.primaryXYStandardDeviationCoefficient = primaryXYStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient = secondaryXYStandardDeviationCoefficient;
    this.primaryThetaStandardDeviationCoefficient = primaryThetaStandardDeviationCoefficient;
  }

  private CameraType(
      double horizontalFOV,
      double verticalFOV,
      double xyStandardDeviationCoefficient,
      double thetaStandardDeviationCoefficient) {
    this.horizontalFOV = horizontalFOV;
    this.verticalFOV = verticalFOV;
    this.primaryXYStandardDeviationCoefficient = xyStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient = xyStandardDeviationCoefficient;
    this.primaryThetaStandardDeviationCoefficient = thetaStandardDeviationCoefficient;
  }

  private CameraType() {
    this.horizontalFOV = 0.0;
    this.verticalFOV = 0.0;
    this.primaryXYStandardDeviationCoefficient = 0.0;
    this.secondaryXYStandardDeviationCoefficient = 0.0;
    this.primaryThetaStandardDeviationCoefficient = 0.0;
  }

  public static final double BLINK_TIME = 0.067;

  private static class Limelight2PlusConstants {
    private static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    private static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    private static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    private static final double MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    private static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
  }

  private static class Limelight3Constants {
    private static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    private static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    private static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    private static final double MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    private static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
  }

  private static class Limelight3GConstants {
    private static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    private static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    private static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    private static final double MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    private static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.00015;
  }

  private static class Limelight4Constants {
    private static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    private static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    private static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    private static final double MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT = 10;
    private static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.00015;
  }

  private static class ThriftyCamConstants {
    private static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    private static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    private static final double SINGLETAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    private static final double THETA_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    private static final double MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.00015;
    private static final int WIDTH = 1600;
    private static final int HEIGHT = 1304;
  }
}
