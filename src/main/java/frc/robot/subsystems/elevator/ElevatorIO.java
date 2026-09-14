package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;


/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorAngle = 0.0;
        public double elevatorSetpoint = 0.0;
        public double elevatorStatorCurrent = 0.0;
        public double elevatorSupplyCurrent = 0.0;
        public double elevatorVoltage = 0.0;
        public double elevatorMotorTemp = 0.0;
        public boolean elevatorMotorConnected = false;
        public boolean elevatorEncoderConnected = false;
        public double elevatorSpeed = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setElevatorSetpoint(double position) {}
    
} 