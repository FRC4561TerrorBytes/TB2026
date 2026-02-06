package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IndexerIOReal implements IndexerIO {

  private TalonFX indexerLeftMotor = new TalonFX(Constants.INDEXER_LEFT_MOTOR_ID);
  private final StatusSignal<Current> indexerLeftStatorCurrent;
  private final StatusSignal<Current> indexerLeftSupplyCurrent;
  private final StatusSignal<AngularVelocity> indexerLeftSpeed;
  private final StatusSignal<Voltage> indexerLeftVoltage;
  private final StatusSignal<Temperature> indexerLeftTemp;

  private TalonFX indexerRightMotor = new TalonFX(Constants.INDEXER_RIGHT_MOTOR_ID);
  private TalonFX fuelKicker = new TalonFX(Constants.FUEL_KICKER_MOTOR_ID);
}
