package frc.robot.subsystems.indexer;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngularMomentumUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IndexerIOReal implements IndexerIO {

  private TalonFX indexerMotor = new TalonFX(Constants.INDEXER_MOTOR_ID);
  private final StatusSignal<Current> IndexerStatorCurrent;
  private final StatusSignal<Current> IndexerSupplyCurrent;
  private final StatusSignal<AngularVelocity> IndexerSpeed;
  private final StatusSignal<Voltage> IndexerVoltage;
  private final StatusSignal<Temperature> IndexerTemp;

  public IndexerIOReal() {

    var indexerConfig = new TalonFXConfiguration();
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerConfig.CurrentLimits.StatorCurrentLimit = Constants.INDEXER_STATOR_CURRENT_LIMIT;
    indexerConfig.CurrentLimits.SupplyCurrentLimit = Constants.INDEXER_SUPPLY_CURRENT_LIMIT;
    indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(indexerConfig, 0.25));

    IndexerStatorCurrent = indexerMotor.getStatorCurrent();
    IndexerSupplyCurrent = indexerMotor.getSupplyCurrent();
    IndexerSpeed = indexerMotor.getVelocity();
    IndexerVoltage = indexerMotor.getMotorVoltage();
    IndexerTemp = indexerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        IndexerStatorCurrent,
        IndexerSupplyCurrent,
        IndexerSpeed,
        IndexerVoltage,
        IndexerTemp);

    ParentDevice.optimizeBusUtilizationForAll(indexerMotor);
  }

  public void updateInputs(IndexerIOInputs inputs){
    var indexerStatus =
        BaseStatusSignal.refreshAll(
            IndexerStatorCurrent,
            IndexerSupplyCurrent,
            IndexerSpeed,
            IndexerVoltage,
            IndexerTemp);

    inputs.indexerVelocity = indexerMotor.getVelocity().getValueAsDouble();
    inputs.indexerCurrentAmps = IndexerStatorCurrent.getValueAsDouble();
    inputs.indexerVoltage = indexerMotor.getMotorVoltage().getValueAsDouble();
    inputs.indexerConnected = indexerStatus.isOK();
  }

  public void setOutput(double speed){
    indexerMotor.set(speed);
  }
}