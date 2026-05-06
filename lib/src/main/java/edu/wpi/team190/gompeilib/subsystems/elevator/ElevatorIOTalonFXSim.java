package edu.wpi.team190.gompeilib.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;

public class ElevatorIOTalonFXSim extends ElevatorIOTalonFX {
  private final ElevatorSim elevatorSim;

  private final TalonFXSimState elevatorController;

  public ElevatorIOTalonFXSim(ElevatorConstants constants) {
    super(constants);
    elevatorSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                constants.elevatorParameters.ELEVATOR_MOTOR_CONFIG(),
                constants.elevatorParameters.CARRIAGE_MASS_KG(),
                constants.drumRadius,
                constants.elevatorGearRatio),
            constants.elevatorParameters.ELEVATOR_MOTOR_CONFIG(),
            constants.elevatorParameters.MIN_HEIGHT().in(Meters),
            constants.elevatorParameters.MAX_HEIGHT().in(Meters),
            true,
            constants.elevatorParameters.MIN_HEIGHT().in(Meters));

    elevatorController = super.talonFX.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double elevatorVoltage = elevatorController.getMotorVoltage();

    elevatorSim.setInputVoltage(elevatorVoltage);

    elevatorSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition =
        Angle.ofBaseUnits(
            elevatorSim.getPositionMeters() * constants.elevatorGearRatio * constants.drumRadius,
            Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            elevatorSim.getVelocityMetersPerSecond()
                * constants.elevatorGearRatio
                * constants.drumRadius,
            RadiansPerSecond);
    elevatorController.setRawRotorPosition(rotorPosition);
    elevatorController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
