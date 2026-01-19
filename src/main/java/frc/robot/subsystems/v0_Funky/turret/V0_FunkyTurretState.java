package frc.robot.subsystems.v0_Funky.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Getter
@NoArgsConstructor
public enum V0_FunkyTurretState {
  IDLE,
  CHARACTERIZING,
  CLOSED_LOOP_ABSOLUTE_POSITION(new Rotation2d()),
  OPEN_LOOP_VOLTAGE_CONTROL(0.0),
  CLOSED_LOOP_RELATIVE_POSITION(new Rotation2d());

  @Setter private Rotation2d rotation;
  @Setter private double voltage;

  V0_FunkyTurretState(Rotation2d rotation) {
    this.rotation = rotation;
  }

  V0_FunkyTurretState(Double voltage) {
    this.voltage = voltage;
  }
}
