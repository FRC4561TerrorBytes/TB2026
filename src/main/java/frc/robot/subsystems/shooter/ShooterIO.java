package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public boolean flywheelLeftConnected = false;
        public double flywheelLeftVelocity = 0.0;
        public double flywheelLeftVoltage = 0.0;
        public double flywheelLeftCurrent = 0.0;
        public double flywheelLeftSetpoint = 0.0;
        public boolean flywheelRightConnected = false;
        public double flywheelRightVelocity = 0.0;
        public double flywheelRightVoltage = 0.0;
        public double flywheelRightCurrent = 0.0;
        public double flywheelRightSetpoint = 0.0;
        public boolean hoodConnected = false;
        public double hoodVelocity = 0.0;
        public double hoodVoltage = 0.0;
        public double hoodCurrent = 0.0;
        public double hoodRelativePosition = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setFlywheelLeftVoltage(double voltage){}

    public default void setFlywheelRightVoltage(double voltage){}

    public default void setLeftFlywheelSpeed(double speed){}

    public default void setRightFlywheelSpeed(double speed){}

    public default boolean leftFlywheelUpToSpeed(double mps) {return false;}

    public default boolean rightFlywheelUpToSpeed(double mps) {return false;}
    
    public default void setHoodAngle(double angle){}

}

/*

       X  |       |
          |    O  |X
    ---------------------
          |       |
      X   |  X    | O
    ---------------------
          |       | 
         O|    X  |  O

    -Tyer + Tyler 🦖
    bleh
*/