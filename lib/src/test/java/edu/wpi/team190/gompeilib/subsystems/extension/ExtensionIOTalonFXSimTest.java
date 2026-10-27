package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
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
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

@SuppressWarnings("unchecked") // Mockito mocks of generic types
public class ExtensionIOTalonFXSimTest {
  private ExtensionConstants constants;

  @BeforeEach
  public void setUp() {
    org.wpilib.hardware.hal.HAL.initialize();
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    DCMotor motor = DCMotor.getNeo550(1);
    ExtensionConstants.ExtensionParameters params =
        ExtensionConstants.ExtensionParameters.builder()
            .withEXTENSION_MOTOR_CONFIG(motor)
            .withCARRIAGE_MASS_KG(15.0)
            .withMIN_LENGTH(Units.Meters.of(0.0))
            .withMAX_LENGTH(Units.Meters.of(1.5))
            .withNUM_MOTORS(1)
            .build();

    constants =
        ExtensionConstants.builder()
            .withLeaderCANID(5)
            .withLeaderInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withCanBus(new CANBus("rio"))
            .withExtensionGearRatio(10.0)
            .withDrumRadius(0.02)
            .withExtensionSupplyCurrentLimit(40.0)
            .withExtensionStatorCurrentLimit(40.0)
            .withExtensionParameters(params)
            .withSlot0Gains(
                Gains.builder()
                    .withPrefix("slot0")
                    .withKP(1.0)
                    .withKI(0.0)
                    .withKD(0.1)
                    .withKS(0.01)
                    .withKV(0.01)
                    .withKA(0.01)
                    .withKG(0.1)
                    .build())
            .withSlot1Gains(
                Gains.builder().withPrefix("slot1").withKP(2.0).withKI(0.0).withKD(0.2).build())
            .withSlot2Gains(
                Gains.builder().withPrefix("slot2").withKP(3.0).withKI(0.0).withKD(0.3).build())
            .withConstraints(
                LinearConstraints.builder()
                    .withPrefix("constraints")
                    .withMaxVelocity(Units.MetersPerSecond.of(2.0))
                    .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(2.0))
                    .withGoalTolerance(Units.Meters.of(0.05))
                    .build())
            .withVerticalGravity(false)
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .withHeightOffsetStep(Units.Meters.of(0.05))
            .build();
  }

  @Test
  public void testExtensionIOTalonFXSim() {
    TalonFXConfigurator configurator = mock(TalonFXConfigurator.class);
    when(configurator.apply(any(TalonFXConfiguration.class))).thenReturn(StatusCode.OK);
    when(configurator.apply(any(TalonFXConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.OK);

    StatusSignal<Angle> positionRotations = mock(StatusSignal.class);
    StatusSignal<AngularVelocity> velocityRotationsPerSecond = mock(StatusSignal.class);
    StatusSignal<AngularAcceleration> accelerationRotationsPerSecondPerSecond =
        mock(StatusSignal.class);
    StatusSignal<Voltage> appliedVolts = mock(StatusSignal.class);
    StatusSignal<Current> supplyCurrentAmps = mock(StatusSignal.class);
    StatusSignal<Current> torqueCurrentAmps = mock(StatusSignal.class);
    StatusSignal<Temperature> temperatureCelsius = mock(StatusSignal.class);
    StatusSignal<Double> positionSetpointRotations = mock(StatusSignal.class);
    StatusSignal<Double> positionErrorRotations = mock(StatusSignal.class);
    StatusSignal<Double> velocitySetpointRotations = mock(StatusSignal.class);
    StatusSignal<Integer> closedLoopSlot = mock(StatusSignal.class);

    when(positionRotations.getValue()).thenReturn(Units.Rotations.of(10.0));
    when(positionRotations.getValueAsDouble()).thenReturn(10.0);
    when(velocityRotationsPerSecond.getValue()).thenReturn(Units.RotationsPerSecond.of(1.5));
    when(velocityRotationsPerSecond.getValueAsDouble()).thenReturn(1.5);
    when(accelerationRotationsPerSecondPerSecond.getValue())
        .thenReturn(Units.RotationsPerSecondPerSecond.of(0.5));
    when(appliedVolts.getValueAsDouble()).thenReturn(6.0);
    when(appliedVolts.getValue()).thenReturn(Units.Volts.of(6.0));
    when(supplyCurrentAmps.getValueAsDouble()).thenReturn(15.0);
    when(torqueCurrentAmps.getValueAsDouble()).thenReturn(12.0);
    when(temperatureCelsius.getValueAsDouble()).thenReturn(45.0);
    when(positionSetpointRotations.getValueAsDouble()).thenReturn(0.5);
    when(positionErrorRotations.getValueAsDouble()).thenReturn(0.01);
    when(velocitySetpointRotations.getValueAsDouble()).thenReturn(0.75);
    when(closedLoopSlot.getValue()).thenReturn(0);

    TalonFXSimState simState = mock(TalonFXSimState.class);
    when(simState.getMotorVoltage()).thenReturn(6.0);

    try (MockedConstruction<TalonFX> _ =
            mockConstruction(
                TalonFX.class,
                (mock, context) -> {
                  when(mock.getConfigurator()).thenReturn(configurator);
                  when(mock.getPosition()).thenReturn(positionRotations);
                  when(mock.getVelocity()).thenReturn(velocityRotationsPerSecond);
                  when(mock.getAcceleration()).thenReturn(accelerationRotationsPerSecondPerSecond);
                  when(mock.getMotorVoltage()).thenReturn(appliedVolts);
                  when(mock.getSupplyCurrent()).thenReturn(supplyCurrentAmps);
                  when(mock.getTorqueCurrent()).thenReturn(torqueCurrentAmps);
                  when(mock.getDeviceTemp()).thenReturn(temperatureCelsius);
                  when(mock.getClosedLoopReference()).thenReturn(positionSetpointRotations);
                  when(mock.getClosedLoopError()).thenReturn(positionErrorRotations);
                  when(mock.getClosedLoopReferenceSlope()).thenReturn(velocitySetpointRotations);
                  when(mock.getClosedLoopSlot()).thenReturn(closedLoopSlot);
                  when(mock.getSimState()).thenReturn(simState);
                });
        MockedStatic<BaseStatusSignal> mockBss = mockStatic(BaseStatusSignal.class)) {

      mockBss
          .when(
              () ->
                  BaseStatusSignal.setUpdateFrequencyForAll(
                      anyDouble(), any(BaseStatusSignal[].class)))
          .thenReturn(null);

      ExtensionIOTalonFXSim sim = new ExtensionIOTalonFXSim(constants);
      ExtensionIO.ExtensionIOInputs inputs = new ExtensionIO.ExtensionIOInputs();

      sim.updateInputs(inputs);

      assertEquals(10.0, inputs.position.in(Units.Meters), 0.01);
      assertEquals(1.5, inputs.velocity.in(Units.MetersPerSecond), 0.01);
      assertEquals(0.75, inputs.linearVelocitySetpoint.in(Units.MetersPerSecond), 0.01);

      verify(simState).setSupplyVoltage(anyDouble());
      verify(simState).setRawRotorPosition(any(Angle.class));
      verify(simState).setRotorVelocity(any(AngularVelocity.class));
    }
  }
}
