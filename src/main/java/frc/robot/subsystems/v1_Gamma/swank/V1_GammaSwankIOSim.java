package frc.robot.subsystems.v1_Gamma.swank;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.first.math.system.plant.DCMotor;

public class V1_GammaSwankIOSim implements V1_GammaSwankIO {

    private final DCMotorSim motorSim;
    private double appliedVolts = 0.0;

    public V1_GammaSwankIOSim() {
        motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                V1_GammaSwankConstants.DRIVE_INERTIA,
                V1_GammaSwankConstants.GEAR_RATIO),
            DCMotor.getKrakenX60(1));
    }

    @Override
    public void updateInputs(V1_GammaSwankIOInputs inputs) {
        motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
        motorSim.update(GompeiLib.getLoopPeriod());

        inputs.position = new Rotation2d(motorSim.getAngularPosition());
        inputs.velocity = RadiansPerSecond.of(motorSim.getAngularVelocityRadPerSec());
        inputs.appliedVolts = Volts.of(appliedVolts);
        inputs.supplyCurrent = Amps.of(motorSim.getCurrentDrawAmps());
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
    }
}
