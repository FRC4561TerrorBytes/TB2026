package frc.robot.subsystems.turret.Turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

public interface TurretIO {
        @AutoLog
    public static class TurretIOInputs{
        public boolean turretConnected = false;
        public boolean encoderConnected = false;
        public double turretAngle = 0.0;
        public double turretCurrentAmps = 0.0;
        public double turretVoltage = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setTurretPosition(double position) {}

    public default void setTurretVoltage(double voltage){}
}