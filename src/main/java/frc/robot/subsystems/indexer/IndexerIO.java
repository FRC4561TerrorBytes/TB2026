package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {

    public double indexerRightVelocity = 0.0;
    public double indexerRightCurrentAmps = 0.0;
    public double indexerRightVoltage = 0.0;
    public boolean indexerRightConnected = true;

    public double indexerLeftVelocity = 0.0;
    public double indexerLeftCurrentAmps = 0.0;
    public double indexerLeftVoltage = 0.0;
    public boolean indexerLeftConnected = true;

    public double fuelKickerVelocity = 0.0;
    public double fuelKickerCurrentAmps = 0.0;
    public double fuelKickerVoltage = 0.0;
    public boolean fuelKickerConnected = true;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setIndexerOutput(double speed) {}
  public default void setFuelKickerOutput(double speed) {}

  
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