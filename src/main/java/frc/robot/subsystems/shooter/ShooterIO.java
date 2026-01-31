package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public boolean flywheelsConnected = false;
        public double flywheelsVelocity = 0.0;
        public double flywheelsVoltage = 0.0;
        public double flywheelsCurrent = 0.0;
        public boolean hoodConnected = false;
        public double hoodVelocity = 0.0;
        public double hoodVoltage = 0.0;
        public double hoodCurrent = 0.0;
        public double hoodAngle = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setFlywheelsVoltage(double voltage){}
    
    public default void setHoodPosition(double position){}

}