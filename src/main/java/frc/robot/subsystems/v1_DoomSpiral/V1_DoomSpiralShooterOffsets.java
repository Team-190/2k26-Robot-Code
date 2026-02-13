package frc.robot.subsystems.v1_DoomSpiral;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;
import lombok.Setter;

public class V1_DoomSpiralShooterOffsets {
    
    @Getter @Setter private double flywheelVelocityOffsetRPS;
    @Getter @Setter private Rotation2d hoodAngleOffsetRotations;

    public V1_DoomSpiralShooterOffsets(double flywheelOffset, Rotation2d hoodOffset) {
        flywheelVelocityOffsetRPS = flywheelOffset;
        hoodAngleOffsetRotations = hoodOffset;
    }

}
