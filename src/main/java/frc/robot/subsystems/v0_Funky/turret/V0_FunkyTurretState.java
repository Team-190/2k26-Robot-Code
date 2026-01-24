package frc.robot.subsystems.v0_Funky.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@NoArgsConstructor
public enum V0_FunkyTurretState {
  IDLE,
  CLOSED_LOOP_POSITION_CONTROL(new Rotation2d()),
  OPEN_LOOP_VOLTAGE_CONTROL(0.0),
  CLOSED_LOOP_AUTO_AIM_CONTROL(new Translation2d());

  @Getter @Setter private Rotation2d rotation;
  @Getter @Setter private double voltage;
  @Getter @Setter private Translation2d translation;

  V0_FunkyTurretState(Rotation2d rotation) {
    this.rotation = rotation;
  }

  V0_FunkyTurretState(double voltage) {
    this.voltage = voltage;
  }

  V0_FunkyTurretState(Translation2d translation) {
    this.translation = translation;
  }
}
