package frc.robot.subsystems.v1_Gamma.spindexer;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;

import static edu.wpi.first.units.Units.*;

public class V1_GammaSpindexerIOTalonFXSim extends V1_GammaSpindexerIOTalonFX { //is this even right?

    private final DCMotorSim motorSim;
    private TalonFXSimState talonFXSim;
    private Voltage motorVoltage;

    public V1_GammaSpindexerIOTalonFXSim() {
        talonFXSim = spindexerMotor.getSimState();
        motorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                V1_GammaSpindexerConstants.MOTOR_CONFIG,
                                V1_GammaSpindexerConstants.MOMENT_OF_INERTIA,
                                V1_GammaSpindexerConstants.GEAR_RATIO),
                        V1_GammaSpindexerConstants.MOTOR_CONFIG);

        talonFXSim.Orientation = V1_GammaSpindexerConstants.SPINDEXER_ORIENTATION;
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    @Trace
    public void updateInputs(V1_GammaSpindexerIOInputs inputs) {
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        motorVoltage = talonFXSim.getMotorVoltageMeasure();
        motorSim.setInputVoltage(motorVoltage.in(Volts));
        motorSim.update(GompeiLib.getLoopPeriod());

        talonFXSim.setRawRotorPosition(motorSim.getAngularPosition().times(V1_GammaSpindexerConstants.GEAR_RATIO));
        talonFXSim.setRotorVelocity(motorSim.getAngularVelocity().times(V1_GammaSpindexerConstants.GEAR_RATIO));
    }

    @Override
    public void setVoltage(double volts) {
        //no clue what goes here
    }
}
