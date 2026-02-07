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
  private final StatusSignal<Current> IndexerLeftStatorCurrent;
  private final StatusSignal<Current> IndexerLeftSupplyCurrent;
  private final StatusSignal<AngularVelocity> IndexerLeftSpeed;
  private final StatusSignal<Voltage> IndexerLeftVoltage;
  private final StatusSignal<Temperature> IndexerLeftTemp;

  private TalonFX indexerRightMotor = new TalonFX(Constants.INDEXER_RIGHT_MOTOR_ID);
  private final StatusSignal<Current> IndexerRightStatorCurrent;
  private final StatusSignal<Current> IndexerRightSupplyCurrent;
  private final StatusSignal<AngularVelocity> IndexerRightSpeed;
  private final StatusSignal<Voltage> IndexerRightVoltage;
  private final StatusSignal<Temperature> IndexerRightTemp;

  private TalonFX fuelKickerMotor = new TalonFX(Constants.FUEL_KICKER_MOTOR_ID);
  private final StatusSignal<Current> FuelKickerStatorCurrent;
  private final StatusSignal<Current> FuelKickerSupplyCurrent;
  private final StatusSignal<AngularVelocity> FuelKickerSpeed;
  private final StatusSignal<Voltage> FuelKickerVoltage;
  private final StatusSignal<Temperature> FuelKickerTemp;

  public IndexerIOReal() {

    var indexerRightConfig = new TalonFXConfiguration();
    indexerRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerRightConfig.CurrentLimits.StatorCurrentLimit = Constants.INDEXER_STATOR_CURRENT_LIMIT;
    indexerRightConfig.CurrentLimits.SupplyCurrentLimit = Constants.INDEXER_SUPPLY_CURRENT_LIMIT;
    indexerRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> indexerRightMotor.getConfigurator().apply(indexerRightConfig, 0.25));

    IndexerRightStatorCurrent = indexerRightMotor.getStatorCurrent();
    IndexerRightSupplyCurrent = indexerRightMotor.getSupplyCurrent();
    IndexerRightSpeed = indexerRightMotor.getVelocity();
    IndexerRightVoltage = indexerRightMotor.getMotorVoltage();
    IndexerRightTemp = indexerRightMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        IndexerRightStatorCurrent,
        IndexerRightSupplyCurrent,
        IndexerRightSpeed,
        IndexerRightVoltage,
        IndexerRightTemp);

    ParentDevice.optimizeBusUtilizationForAll(indexerRightMotor);

    var indexerLeftConfig = new TalonFXConfiguration();
    indexerLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerLeftConfig.CurrentLimits.StatorCurrentLimit = Constants.INDEXER_STATOR_CURRENT_LIMIT;
    indexerLeftConfig.CurrentLimits.SupplyCurrentLimit = Constants.INDEXER_SUPPLY_CURRENT_LIMIT;
    indexerLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> indexerLeftMotor.getConfigurator().apply(indexerLeftConfig, 0.25));

    IndexerLeftStatorCurrent = indexerLeftMotor.getStatorCurrent();
    IndexerLeftSupplyCurrent = indexerLeftMotor.getSupplyCurrent();
    IndexerLeftSpeed = indexerLeftMotor.getVelocity();
    IndexerLeftVoltage = indexerLeftMotor.getMotorVoltage();
    IndexerLeftTemp = indexerLeftMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        IndexerLeftStatorCurrent,
        IndexerLeftSupplyCurrent,
        IndexerLeftSpeed,
        IndexerLeftVoltage,
        IndexerLeftTemp);

    ParentDevice.optimizeBusUtilizationForAll(indexerLeftMotor);

    var fuelKickerConfig = new TalonFXConfiguration();
    fuelKickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    fuelKickerConfig.CurrentLimits.StatorCurrentLimit = Constants.INDEXER_STATOR_CURRENT_LIMIT;
    fuelKickerConfig.CurrentLimits.SupplyCurrentLimit = Constants.INDEXER_SUPPLY_CURRENT_LIMIT;
    fuelKickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    fuelKickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> fuelKickerMotor.getConfigurator().apply(fuelKickerConfig, 0.25));

    FuelKickerStatorCurrent = fuelKickerMotor.getStatorCurrent();
    FuelKickerSupplyCurrent = fuelKickerMotor.getSupplyCurrent();
    FuelKickerSpeed = fuelKickerMotor.getVelocity();
    FuelKickerVoltage = fuelKickerMotor.getMotorVoltage();
    FuelKickerTemp = fuelKickerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        FuelKickerStatorCurrent,
        FuelKickerSupplyCurrent,
        FuelKickerSpeed,
        FuelKickerVoltage,
        FuelKickerTemp);

    ParentDevice.optimizeBusUtilizationForAll(fuelKickerMotor);
  }

  public void updateInputs(IndexerIOInputs inputs){
    var fuelKickerStatus =
        BaseStatusSignal.refreshAll(
            FuelKickerStatorCurrent,
            FuelKickerSupplyCurrent,
            FuelKickerSpeed,
            FuelKickerVoltage,
            FuelKickerTemp);

    inputs.fuelKickerVelocity = fuelKickerMotor.getVelocity().getValueAsDouble();
    inputs.fuelKickerCurrentAmps = FuelKickerStatorCurrent.getValueAsDouble();
    inputs.fuelKickerVoltage = fuelKickerMotor.getMotorVoltage().getValueAsDouble();
    inputs.fuelKickerConnected = fuelKickerStatus.isOK();
  }

  public void setThroughput(double indexerSpeed, double kickerSpeed){
    indexerRightMotor.set(-indexerSpeed); //this is a guess, but it might be the other side
    indexerLeftMotor.set(indexerSpeed);
    fuelKickerMotor.set(kickerSpeed);
  }
}