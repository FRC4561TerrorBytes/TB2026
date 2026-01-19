package frc.robot.subsystems.alphashooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlphaShooter extends SubsystemBase{
    // Hardware objects
    private final TalonFX upperTalon;
    private final TalonFX lowerTalon;

    // Voltage control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public AlphaShooter() {
        upperTalon = new TalonFX(34, CANBus.roboRIO());
        lowerTalon = new TalonFX(31, CANBus.roboRIO());

        var shooterConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(40))
                .withSupplyCurrentLimitEnable(true)
        );


        tryUntilOk(5, () -> upperTalon.getConfigurator().apply(shooterConfig, 0.25));
        tryUntilOk(5, () -> lowerTalon.getConfigurator().apply(shooterConfig, 0.25));

        lowerTalon.setControl(new Follower(upperTalon.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void stopShooter() {
        upperTalon.setControl(voltageRequest.withOutput(0));
    }

    public void shoot() {
        upperTalon.setControl(voltageRequest.withOutput(-9));
    }
    
}
