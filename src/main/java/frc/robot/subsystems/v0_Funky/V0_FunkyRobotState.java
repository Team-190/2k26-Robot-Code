package frc.robot.subsystems.v0_Funky;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.team190.gompeilib.core.robot.RobotState;
import frc.robot.util.NTPrefixes;
import lombok.Getter;

public class V0_FunkyRobotState implements RobotState {
    @AutoLogOutput(key = NTPrefixes.ROBOT_STATE+"Hood/Score Angle") @Getter private static Rotation2d scoreAngle = new Rotation2d();
    @AutoLogOutput(key = NTPrefixes.ROBOT_STATE+"Hood/Feed Angle") @Getter private static Rotation2d feedAngle = new Rotation2d();
}
