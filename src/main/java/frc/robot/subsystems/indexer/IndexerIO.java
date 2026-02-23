package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {

    public boolean indexerLeftConnected = true;
    public double indexerLeftVelocity = 0.0;
    public double indexerLeftCurrentAmps = 0.0;
    public double indexerLeftVoltage = 0.0;
    public double indexerLeftTemp = 0.0;
    
    public boolean indexerRightConnected = true;
    public double indexerRightVelocity = 0.0;
    public double indexerRightCurrentAmps = 0.0;
    public double indexerRightVoltage = 0.0;
    public double indexerRightTemp = 0.0;

    public double fuelKickerVelocity = 0.0;
    public double fuelKickerCurrentAmps = 0.0;
    public double fuelKickerVoltage = 0.0;
    public boolean fuelKickerConnected = true;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setLeftIndexerVoltage(double voltage) {}
  public default void setRightIndexerVoltage(double voltage) {}
  public default void setFuelKickerVoltage(double voltage) {}

  
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