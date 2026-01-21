package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

public interface FlywheelsIO {
        @AutoLog
    public class FlywheelsIOInputs {
        public boolean flywheelsConnected = false;
        public double flywheelsVelocity = 0.0;
        public double flywheelsVoltage = 0.0;
        public double flywheelsCurrent = 0.0;
    }

    public default void updateInputs(FlywheelsIOInputs inputs) {}

    public default void setFlywheelsVoltage(double voltage){}
}
