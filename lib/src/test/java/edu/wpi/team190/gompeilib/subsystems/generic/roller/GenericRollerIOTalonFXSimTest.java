package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class GenericRollerIOTalonFXSimTest {
  private GenericRollerConstants constants;

  @BeforeEach
  public void setUp() {
    edu.wpi.first.hal.HAL.initialize(500, 0);
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    constants =
        GenericRollerConstants.builder()
            .withLeaderCANID(1)
            .withLeaderInvertedValue(InvertedValue.Clockwise_Positive)
            .withCurrentLimits(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(30.0)
                    .withStatorCurrentLimit(30.0)
                    .build())
            .withRollerGearbox(DCMotor.getNeo550(1))
            .withRollerMotorGearRatio(5.0)
            .withMomentOfInertia(Units.KilogramSquareMeters.of(0.001))
            .withNeutralMode(NeutralModeValue.Brake)
            .withCanBus(new CANBus("rio"))
            .withEnableFOC(false)
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .build();
  }

  @Test
  public void testRollerIOTalonFXSim() {
    TalonFXConfigurator configurator = mock(TalonFXConfigurator.class);
    when(configurator.apply(any(TalonFXConfiguration.class))).thenReturn(StatusCode.OK);
    when(configurator.apply(any(TalonFXConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.OK);

    StatusSignal<Angle> positionRotations = mock(StatusSignal.class);
    StatusSignal<AngularVelocity> velocityRotationsPerSecond = mock(StatusSignal.class);
    StatusSignal<Voltage> appliedVolts = mock(StatusSignal.class);
    StatusSignal<Current> supplyCurrentAmps = mock(StatusSignal.class);
    StatusSignal<Current> torqueCurrentAmps = mock(StatusSignal.class);
    StatusSignal<Temperature> temperatureCelsius = mock(StatusSignal.class);

    when(positionRotations.getValueAsDouble()).thenReturn(10.0);
    when(velocityRotationsPerSecond.getValue()).thenReturn(Units.RotationsPerSecond.of(5.0));
    when(velocityRotationsPerSecond.getValueAsDouble()).thenReturn(5.0);
    when(appliedVolts.getValueAsDouble()).thenReturn(6.0);
    when(appliedVolts.getValue()).thenReturn(Units.Volts.of(6.0));
    when(supplyCurrentAmps.getValueAsDouble()).thenReturn(15.0);
    when(torqueCurrentAmps.getValueAsDouble()).thenReturn(12.0);
    when(temperatureCelsius.getValueAsDouble()).thenReturn(45.0);

    TalonFXSimState simState = mock(TalonFXSimState.class);
    when(simState.getMotorVoltage()).thenReturn(6.0);

    try (MockedConstruction<TalonFX> mockTalon =
            mockConstruction(
                TalonFX.class,
                (mock, context) -> {
                  when(mock.getConfigurator()).thenReturn(configurator);
                  when(mock.getPosition()).thenReturn(positionRotations);
                  when(mock.getVelocity()).thenReturn(velocityRotationsPerSecond);
                  when(mock.getMotorVoltage()).thenReturn(appliedVolts);
                  when(mock.getSupplyCurrent()).thenReturn(supplyCurrentAmps);
                  when(mock.getTorqueCurrent()).thenReturn(torqueCurrentAmps);
                  when(mock.getDeviceTemp()).thenReturn(temperatureCelsius);
                  when(mock.getSimState()).thenReturn(simState);
                  when(mock.getNetwork()).thenReturn(new CANBus("rio"));
                  when(mock.getDeviceID()).thenReturn(1);
                });
        MockedStatic<BaseStatusSignal> mockBss = mockStatic(BaseStatusSignal.class)) {

      mockBss
          .when(
              () ->
                  BaseStatusSignal.setUpdateFrequencyForAll(
                      anyDouble(), any(BaseStatusSignal[].class)))
          .thenReturn(null);

      GenericRollerIOTalonFXSim sim = new GenericRollerIOTalonFXSim(constants);
      GenericRollerIO.GenericRollerIOInputs inputs = new GenericRollerIO.GenericRollerIOInputs();

      sim.updateInputs(inputs);

      assertEquals(10.0, inputs.position.getRotations(), 0.01);
      assertEquals(5.0, inputs.velocity.in(Units.RotationsPerSecond), 0.01);

      verify(simState).setSupplyVoltage(anyDouble());
      verify(simState).setRawRotorPosition(any(Angle.class));
      verify(simState).setRotorVelocity(any(AngularVelocity.class));
    }
  }
}
