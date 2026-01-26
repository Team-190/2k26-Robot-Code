package frc.robot.subsystems.shared.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@NoArgsConstructor
public enum TurretState {
  IDLE,
  CLOSED_LOOP_POSITION_CONTROL(new Rotation2d()),
  OPEN_LOOP_VOLTAGE_CONTROL(0.0),
  CLOSED_LOOP_AUTO_AIM_CONTROL(new Translation2d());

  @Getter @Setter private Rotation2d rotation;
  @Getter @Setter private double voltage;
  @Getter @Setter private Translation2d translation;

  TurretState(Rotation2d rotation) {
    this.rotation = rotation;
  }

  TurretState(double voltage) {
    this.voltage = voltage;
  }

  TurretState(Translation2d translation) {
    this.translation = translation;
  }
}
