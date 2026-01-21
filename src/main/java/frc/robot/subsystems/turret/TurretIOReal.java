package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.turret.TurretIO;

public class TurretIOReal implements TurretIO {
    private final TalonFX turretTalon = new TalonFX(Constants.TURRET_ID);
    private final TalonFX hoodTalon = new TalonFX(Constants.TURRET_HOOD_ID);

    private final StatusSignal<AngularVelocity> turretVelocity;
    private final StatusSignal<Voltage> turretVoltage;
    private final StatusSignal<Current> turretCurrent;

    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodVoltage;
    private final StatusSignal<Current> hoodCurrent;

    public TurretIOReal () { 
        turretVelocity = turretTalon.getVelocity();
        turretVoltage = turretTalon.getMotorVoltage();
        turretCurrent = turretTalon.getStatorCurrent();

        hoodVelocity = hoodTalon.getVelocity();
        hoodVoltage = hoodTalon.getMotorVoltage();
        hoodCurrent = hoodTalon.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
            turretVelocity,
            turretVoltage,
            turretCurrent,
            hoodVelocity,
            hoodVoltage,
            hoodCurrent);
        
        ParentDevice.optimizeBusUtilizationForAll(turretTalon, hoodTalon);
 }
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        var turretStatus = BaseStatusSignal.refreshAll(
            turretVelocity,
            turretVoltage,
            turretCurrent
        );

        var hoodStatus = BaseStatusSignal.refreshAll(
            hoodVelocity,
            hoodVoltage,
            hoodCurrent
        );

        inputs.turretConnected = turretStatus.isOK();
        inputs.hoodConnected = turretStatus.isOK();

        inputs.turretVelocity = turretVelocity.getValueAsDouble();
        inputs.turretVoltage = turretVoltage.getValueAsDouble();
        inputs.turretCurrent = turretCurrent.getValueAsDouble();

        inputs.hoodVelocity = hoodVelocity.getValueAsDouble();
        inputs.hoodVoltage = hoodVoltage.getValueAsDouble();
        inputs.hoodCurrent = hoodCurrent.getValueAsDouble();
    }
}