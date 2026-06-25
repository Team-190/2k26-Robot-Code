package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

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
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class GenericFlywheelIOTalonFXSimTest {
  private GenericFlywheelConstants constants;

  @BeforeEach
  public void setUp() {
    edu.wpi.first.hal.HAL.initialize(500, 0);
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    constants =
        GenericFlywheelConstants.builder()
            .withLeaderCANID(5)
            .withLeaderInversion(InvertedValue.CounterClockwise_Positive)
            .withCanBus(new CANBus("rio"))
            .withEnableFOC(false)
            .withCurrentLimit(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(40.0)
                    .withStatorCurrentLimit(40.0)
                    .build())
            .withMomentOfInertia(0.01)
            .withGearRatio(1.0)
            .withMotorConfig(DCMotor.getNeo550(1))
            .withVoltageGains(
                Gains.fromDoubles()
                    .withPrefix("voltage")
                    .withKP(1.0)
                    .withKI(0.0)
                    .withKD(0.1)
                    .withKS(0.01)
                    .withKV(0.1)
                    .withKA(0.0)
                    .withKG(0.0)
                    .build())
            .withTorqueGains(
                Gains.fromDoubles()
                    .withPrefix("torque")
                    .withKP(0.5)
                    .withKI(0.0)
                    .withKD(0.05)
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKG(0.0)
                    .build())
            .withConstraints(
                AngularVelocityConstraints.fromMeasures()
                    .withPrefix("constraints")
                    .withMaxVelocity(Units.RadiansPerSecond.of(100.0))
                    .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(50.0))
                    .withGoalTolerance(Units.RadiansPerSecond.of(1.0))
                    .build())
            .withVelocityOffsetStep(Units.RadiansPerSecond.of(5.0))
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .build();
  }

  @Test
  public void testGenericFlywheelIOTalonFXSim() {
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
    StatusSignal<Double> velocitySetpointRotationsPerSecond = mock(StatusSignal.class);
    StatusSignal<Double> velocityErrorRotationsPerSecond = mock(StatusSignal.class);
    StatusSignal<Integer> closedLoopSlot = mock(StatusSignal.class);

    when(positionRotations.getValueAsDouble()).thenReturn(10.0);
    when(velocityRotationsPerSecond.getValue()).thenReturn(Units.RotationsPerSecond.of(5.0));
    when(velocityRotationsPerSecond.getValueAsDouble()).thenReturn(5.0);
    when(appliedVolts.getValueAsDouble()).thenReturn(6.0);
    when(appliedVolts.getValue()).thenReturn(Units.Volts.of(6.0));
    when(supplyCurrentAmps.getValueAsDouble()).thenReturn(15.0);
    when(torqueCurrentAmps.getValueAsDouble()).thenReturn(12.0);
    when(temperatureCelsius.getValueAsDouble()).thenReturn(45.0);
    when(velocitySetpointRotationsPerSecond.getValueAsDouble()).thenReturn(5.0);
    when(velocityErrorRotationsPerSecond.getValueAsDouble()).thenReturn(0.1);
    when(closedLoopSlot.getValue()).thenReturn(0);

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
                  when(mock.getClosedLoopReference())
                      .thenReturn(velocitySetpointRotationsPerSecond);
                  when(mock.getClosedLoopError()).thenReturn(velocityErrorRotationsPerSecond);
                  when(mock.getClosedLoopSlot()).thenReturn(closedLoopSlot);
                  when(mock.getSimState()).thenReturn(simState);
                  when(mock.getNetwork()).thenReturn(new CANBus("rio"));
                  when(mock.getDeviceID()).thenReturn(5);
                });
        MockedStatic<BaseStatusSignal> mockBss = mockStatic(BaseStatusSignal.class)) {

      mockBss
          .when(
              () ->
                  BaseStatusSignal.setUpdateFrequencyForAll(
                      anyDouble(), any(BaseStatusSignal[].class)))
          .thenReturn(null);

      GenericFlywheelIOTalonFXSim sim = new GenericFlywheelIOTalonFXSim(constants);
      GenericFlywheelIO.GenericFlywheelIOInputs inputs =
          new GenericFlywheelIO.GenericFlywheelIOInputs();

      sim.updateInputs(inputs);

      assertEquals(10.0, inputs.position.getRotations(), 0.01);
      assertEquals(5.0, inputs.velocity.in(Units.RotationsPerSecond), 0.01);

      verify(simState).setSupplyVoltage(anyDouble());
      verify(simState).setRotorVelocity(any(AngularVelocity.class));
      verify(simState).setRotorAcceleration(any(AngularAcceleration.class));
    }
  }
}
