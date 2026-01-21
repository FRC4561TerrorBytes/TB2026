package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
        @AutoLog
    public static class TurretIOInputs{
        public boolean turretConnected = false;
        public double turretVelocity = 0.0;
        public double turretVoltage = 0.0;
        public double turretCurrent = 0.0;
        public double turretAngle = 0.0;
        public boolean hoodConnected = false;
        public double hoodVelocity = 0.0;
        public double hoodVoltage = 0.0;
        public double hoodCurrent = 0.0;
        public double hoodAngle = 0.0;
        public boolean encoderConnected = false;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setTurretPosition(double position) {}

    public default void setTurretVoltage(double voltage){}
}