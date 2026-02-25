package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public boolean flywheelLeftTopConnected = false;
        public double flywheelLeftTopVelocity = 0.0;
        public double flywheelLeftTopVoltage = 0.0;
        public double flywheelLeftTopCurrent = 0.0;

        public boolean flywheelLeftBottomConnected = false;
        public double flywheelLeftBottomVelocity = 0.0;
        public double flywheelLeftBottomVoltage = 0.0;
        public double flywheelLeftBottomCurrent = 0.0;

        public double flywheelLeftSetpoint = 0.0;


        public boolean flywheelRightTopConnected = false;
        public double flywheelRightTopVelocity = 0.0;
        public double flywheelRightTopVoltage = 0.0;
        public double flywheelRightTopCurrent = 0.0;

        public boolean flywheelRightBottomConnected = false;
        public double flywheelRightBottomVelocity = 0.0;
        public double flywheelRightBottomVoltage = 0.0;
        public double flywheelRightBottomCurrent = 0.0;

        public double flywheelRightSetpoint = 0.0;


        public boolean hoodConnected = false;
        public double hoodVelocity = 0.0;
        public double hoodVoltage = 0.0;
        public double hoodCurrent = 0.0;
        public double hoodRelativePosition = 0.0;
        public double hoodSetpoint = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setLeftFlywheelVoltage(double voltage){}

    public default void setRightFlywheelVoltage(double voltage){}

    public default void setLeftFlywheelSpeed(double velocityRPS){}

    public default void setRightFlywheelSpeed(double velocityRPS){}

    public default boolean leftFlywheelUpToSpeed(double rotationsPerSecond) {return false;}

    public default boolean rightFlywheelUpToSpeed(double rotationsPerSecond) {return false;}
    
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