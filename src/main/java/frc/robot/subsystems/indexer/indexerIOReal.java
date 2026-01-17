package frc.robot.subsystems.indexer;


import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

public class indexerIOReal implements indexerIO{

    private TalonFX indexerMotor = new TalonFX(Constants.INDEXER_MOTOR_ID);
    
}
