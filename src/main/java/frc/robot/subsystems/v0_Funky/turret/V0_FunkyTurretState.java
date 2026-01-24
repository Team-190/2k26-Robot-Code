package frc.robot.subsystems.v0_Funky.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@NoArgsConstructor
public enum V0_FunkyTurretState {
  IDLE,
  CLOSED_LOOP_POSITION_CONTROL(new Rotation2d()),
  OPEN_LOOP_VOLTAGE_CONTROL(0.0),
  CLOSED_LOOP_AUTO_AIM_CONTROL(new Rotation2d());

  @Getter @Setter private Rotation2d rotation;
  @Getter @Setter private double voltage;

  V0_FunkyTurretState(Rotation2d rotation) {
    this.rotation = rotation;
  }

  V0_FunkyTurretState(double voltage) {
    this.voltage = voltage;
  }
}
