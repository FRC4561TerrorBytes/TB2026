package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;

public class AutoShootCommand extends Command {

    public Drive drive;
    public Indexer indexer;
    public Shooter shooter;
    public Intake intake;
    public Extension extension;
    public double distanceToHub;
    private double startTime;

    public double targetAngle;
    public double shootSpeedRPS = 70;

    public AutoShootCommand(Drive drive, Indexer indexer, Shooter shooter, Extension extension, Intake intake) {
        this.drive = drive;
        this.indexer = indexer;
        this.shooter = shooter;
        this.extension = extension;
        this.intake = intake;
        addRequirements(drive, indexer, shooter);
    }
    
    @Override
    public void initialize() {
        //distanceToHub = drive.getPose().getTranslation().getDistance(AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));
        distanceToHub = drive.getDistanceToHub();
        shootSpeedRPS = shooter.getFlywheelShootSpeed(distanceToHub);
        shooter.setFlywheelSpeed(shootSpeedRPS);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        //getting hood angle from the table with interpolation
        double hoodAngleInterpolated = shooter.interpolateHoodAngle(distanceToHub);
        shooter.setHoodAngle(hoodAngleInterpolated);

        double time = System.currentTimeMillis();

        if(shooter.leftFlywheelUpToSpeed(shootSpeedRPS) && shooter.rightFlywheelUpToSpeed(shootSpeedRPS) && shooter.hoodAtSetpoint()){
            
            indexer.setThroughput(0.6, 0.7);
            if(startTime - time> 1000){
                new AgitateBallsCommand(extension, intake);
                startTime = System.currentTimeMillis();
            }
        }
        else{
            indexer.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        indexer.stop();
        extension.setExtensionSetpoint(Constants.EXTENSION_RETRACTED_POSITION);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
