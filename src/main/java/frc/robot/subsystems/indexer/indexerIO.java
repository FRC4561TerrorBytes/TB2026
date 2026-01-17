package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface indexerIO {

    @AutoLog
    public static class indexerIOInputs{

        public double indexerVelocity = 0.0;
        public double indexerCurrentAmps = 0.0;
        public double indexerVoltage = 0.0;
        public boolean indexerConnected = true;
    }

    public default void updateInputs(indexerIOInputs inputs) {}

    public default void setOutput(double speed) {}
}

/* 

          |       |X
     O    |    X  |
    ---------------------
          |       |
        X |   O   |O
    ---------------------
       X  |       |X
          |  O    |
        
    -Sam & Tea🔥
*/