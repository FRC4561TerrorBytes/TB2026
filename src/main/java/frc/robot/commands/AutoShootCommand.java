package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;

public class AutoShootCommand extends Command {

    public Drive drive;
    public Indexer indexer;
    public Shooter shooter;
    public double distanceToHub;

    public double targetAngle;
    public double shootSpeedRPS = 5;

    public AutoShootCommand(Drive drive, Indexer indexer, Shooter shooter) {
        this.drive = drive;
        this.indexer = indexer;
        this.shooter = shooter;
    }
    
    @Override
    public void initialize() {
        distanceToHub = drive.getPose().getTranslation().getDistance(AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));
        shooter.setFlywheelSpeed(shootSpeedRPS);
    }

    @Override
    public void execute() {
        //getting hood angle from the table with interpolation
        double hoodAngleInterpolated = shooter.interpolateHoodAngle(distanceToHub);
        shooter.setHoodAngle(hoodAngleInterpolated);

        if(shooter.leftFlywheelUpToSpeed(shootSpeedRPS) && shooter.rightFlywheelUpToSpeed(shootSpeedRPS) && shooter.hoodAtSetpoint()){
            indexer.setThroughput(0.5, 0.5);
        }
        else{
            indexer.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        indexer.stop();
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
