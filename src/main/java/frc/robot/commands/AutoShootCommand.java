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
    public double targetMPS = 5;

    public AutoShootCommand(Drive drive, Indexer indexer, Shooter shooter) {
        this.drive = drive;
        this.indexer = indexer;
        this.shooter = shooter;
    }
    
    @Override
    public void initialize() {
        distanceToHub = drive.getPose().getTranslation().getDistance(AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));
    }

    @Override
    public void execute() {
        double hoodAngleInterpolated = shooter.interpolateHoodAngle(distanceToHub);
        //shooter.setHood(hoodAngleInterpolated);
        //shooter.setFlywheelsVoltage(targetMPS); - setFlywheelsSpeed instead
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
