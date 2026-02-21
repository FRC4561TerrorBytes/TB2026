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

  private TalonFX indexerLeftMotor = new TalonFX(Constants.INDEXER_LEFT_MOTOR_ID);
  private final StatusSignal<Current> indexerLeftCurrent;
  private final StatusSignal<AngularVelocity> indexerLeftVelocity;
  private final StatusSignal<Voltage> indexerLeftVoltage;
  private final StatusSignal<Temperature> indexerLeftTemp;

  private TalonFX indexerRightMotor = new TalonFX(Constants.INDEXER_RIGHT_MOTOR_ID);
  private final StatusSignal<Current> indexerRightCurrent;
  private final StatusSignal<AngularVelocity> indexerRightVelocity;
  private final StatusSignal<Voltage> indexerRightVoltage;
  private final StatusSignal<Temperature> indexerRightTemp;

  private TalonFX fuelKickerMotor = new TalonFX(Constants.FUEL_KICKER_MOTOR_ID);
  private final StatusSignal<Current> fuelKickerStatorCurrent;
  private final StatusSignal<Current> fuelKickerSupplyCurrent;
  private final StatusSignal<AngularVelocity> fuelKickerSpeed;
  private final StatusSignal<Voltage> fuelKickerVoltage;
  private final StatusSignal<Temperature> fuelKickerTemp;

  public IndexerIOReal() {

    var indexerRightConfig = new TalonFXConfiguration();
    indexerRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerRightConfig.CurrentLimits.StatorCurrentLimit = Constants.INDEXER_STATOR_CURRENT_LIMIT;
    indexerRightConfig.CurrentLimits.SupplyCurrentLimit = Constants.INDEXER_SUPPLY_CURRENT_LIMIT;
    indexerRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> indexerRightMotor.getConfigurator().apply(indexerRightConfig, 0.25));

    indexerRightCurrent = indexerRightMotor.getStatorCurrent();
    indexerRightVelocity = indexerRightMotor.getVelocity();
    indexerRightVoltage = indexerRightMotor.getMotorVoltage();
    indexerRightTemp = indexerRightMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        indexerRightCurrent,
        indexerRightVelocity,
        indexerRightVoltage,
        indexerRightTemp);

    ParentDevice.optimizeBusUtilizationForAll(indexerRightMotor);

    var indexerLeftConfig = new TalonFXConfiguration();
    indexerLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerLeftConfig.CurrentLimits.StatorCurrentLimit = Constants.INDEXER_STATOR_CURRENT_LIMIT;
    indexerLeftConfig.CurrentLimits.SupplyCurrentLimit = Constants.INDEXER_SUPPLY_CURRENT_LIMIT;
    indexerLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> indexerLeftMotor.getConfigurator().apply(indexerLeftConfig, 0.25));

    indexerLeftCurrent = indexerLeftMotor.getStatorCurrent();
    indexerLeftVelocity = indexerLeftMotor.getVelocity();
    indexerLeftVoltage = indexerLeftMotor.getMotorVoltage();
    indexerLeftTemp = indexerLeftMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        indexerLeftCurrent,
        indexerLeftVelocity,
        indexerLeftVoltage,
        indexerLeftTemp);

    ParentDevice.optimizeBusUtilizationForAll(indexerLeftMotor);

    var fuelKickerConfig = new TalonFXConfiguration();
    fuelKickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    fuelKickerConfig.CurrentLimits.StatorCurrentLimit = Constants.INDEXER_STATOR_CURRENT_LIMIT;
    fuelKickerConfig.CurrentLimits.SupplyCurrentLimit = Constants.INDEXER_SUPPLY_CURRENT_LIMIT;
    fuelKickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    fuelKickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> fuelKickerMotor.getConfigurator().apply(fuelKickerConfig, 0.25));

    fuelKickerStatorCurrent = fuelKickerMotor.getStatorCurrent();
    fuelKickerSupplyCurrent = fuelKickerMotor.getSupplyCurrent();
    fuelKickerSpeed = fuelKickerMotor.getVelocity();
    fuelKickerVoltage = fuelKickerMotor.getMotorVoltage();
    fuelKickerTemp = fuelKickerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        fuelKickerStatorCurrent,
        fuelKickerSupplyCurrent,
        fuelKickerSpeed,
        fuelKickerVoltage,
        fuelKickerTemp);

    ParentDevice.optimizeBusUtilizationForAll(fuelKickerMotor);
  }

  public void updateInputs(IndexerIOInputs inputs){
    var leftIndexerStatus = 
      BaseStatusSignal.refreshAll(
        indexerLeftCurrent,
        indexerLeftVelocity,
        indexerLeftVoltage,
        indexerLeftTemp);

    var rightIndexerStatus = 
      BaseStatusSignal.refreshAll(
        indexerRightCurrent,
        indexerRightVelocity,
        indexerRightVoltage,
        indexerRightTemp);
        
    var fuelKickerStatus =
        BaseStatusSignal.refreshAll(
            fuelKickerStatorCurrent,
            fuelKickerSupplyCurrent,
            fuelKickerSpeed,
            fuelKickerVoltage,
            fuelKickerTemp);

    inputs.indexerLeftConnected = leftIndexerStatus.isOK();
    inputs.indexerLeftVelocity = indexerLeftVelocity.getValueAsDouble();
    inputs.indexerLeftCurrentAmps = indexerLeftCurrent.getValueAsDouble();
    inputs.indexerLeftVoltage = indexerLeftMotor.getMotorVoltage().getValueAsDouble();
    inputs.indexerLeftTemp = indexerLeftTemp.getValueAsDouble();

    inputs.indexerRightConnected = rightIndexerStatus.isOK();
    inputs.indexerRightVelocity = indexerRightVelocity.getValueAsDouble();
    inputs.indexerRightCurrentAmps = indexerRightCurrent.getValueAsDouble();
    inputs.indexerRightVoltage = indexerRightMotor.getMotorVoltage().getValueAsDouble();
    inputs.indexerRightTemp = indexerRightTemp.getValueAsDouble();

    inputs.fuelKickerVelocity = fuelKickerMotor.getVelocity().getValueAsDouble();
    inputs.fuelKickerCurrentAmps = fuelKickerStatorCurrent.getValueAsDouble();
    inputs.fuelKickerVoltage = fuelKickerMotor.getMotorVoltage().getValueAsDouble();
    inputs.fuelKickerConnected = fuelKickerStatus.isOK();
  }

  public void setLeftIndexerVoltage(double voltage){
    indexerLeftMotor.set(voltage);
  }
  
  public void setRightIndexerVoltage(double voltage){
    indexerRightMotor.set(voltage);
  }

  public void setFuelKickerVoltage(double voltage){
    fuelKickerMotor.set(voltage);
  }
}