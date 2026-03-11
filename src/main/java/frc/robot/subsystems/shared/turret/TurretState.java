package frc.robot.subsystems.shared.turret;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Voltage;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@NoArgsConstructor
public enum TurretState {
  IDLE,
  CLOSED_LOOP_POSITION_CONTROL(new Rotation2d()),
  OPEN_LOOP_VOLTAGE_CONTROL(Volts.zero()),
  CLOSED_LOOP_AUTO_AIM_CONTROL(new Translation2d());

  @Getter @Setter private Rotation2d rotation;
  @Getter @Setter private Voltage voltage;
  @Getter @Setter private Translation2d translation;

  TurretState(Rotation2d rotation) {
    this.rotation = rotation;
    voltage = Volts.zero();
    translation = new Translation2d();
  }

  TurretState(Voltage voltage) {
    this.voltage = voltage;
    rotation = new Rotation2d();
    translation = new Translation2d();
  }

  TurretState(Translation2d translation) {
    this.translation = translation;
    rotation = new Rotation2d();
    voltage = Volts.zero();
  }
}
