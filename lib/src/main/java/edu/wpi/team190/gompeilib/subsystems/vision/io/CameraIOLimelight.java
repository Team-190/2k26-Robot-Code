package edu.wpi.team190.gompeilib.subsystems.vision.io;

import edu.wpi.team190.gompeilib.core.utility.LimelightHelpers;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants;
import java.util.Arrays;
import lombok.Getter;

public class CameraIOLimelight implements CameraIO {
  @Getter private final String name;

  public CameraIOLimelight(VisionConstants.StaticLimelightConfig config) {
    this.name = "limelight-" + config.key();
  }

  public CameraIOLimelight(VisionConstants.MovingLimelightConfig config) {
    this.name = "limelight-" + config.key();
  }

  @Override
  public void updateInputs(LimelightIOInputs inputs) {
    inputs.mt1PoseEstimate = new PoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue(name));
    inputs.mt2PoseEstimate =
        new PoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name));
    inputs.rawFiducials =
        Arrays.stream(LimelightHelpers.getRawFiducials(name))
            .map(RawFiducial::new)
            .toArray(RawFiducial[]::new);
  }
}
