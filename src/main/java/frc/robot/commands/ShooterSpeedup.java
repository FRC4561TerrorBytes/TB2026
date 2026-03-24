package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;

public class ShooterSpeedup extends Command {
    public Shooter shooter;
    public Drive drive;
    public double distanceToHub;

    public double targetAngle;
    public double shootSpeedRPS = 70;
    PIDController controller;

    public ShooterSpeedup(Shooter shooter, Drive drive) {
        this.shooter = shooter;
        this.drive = drive;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        //distanceToHub = drive.getPose().getTranslation().getDistance(AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));
        distanceToHub = drive.getDistanceToHub();
        shootSpeedRPS = shooter.getFlywheelShootSpeed(distanceToHub);
        shooter.setFlywheelSpeed(shootSpeedRPS);
        //getting hood angle from the table with interpolation
        double hoodAngleInterpolated = shooter.interpolateHoodAngle(distanceToHub);
        shooter.setHoodAngle(hoodAngleInterpolated);
        Logger.recordOutput("DriveTrainFacingHub", controller.atSetpoint());
        Logger.recordOutput("ShooterReady", shooter.leftFlywheelUpToSpeed(shootSpeedRPS) && shooter.rightFlywheelUpToSpeed(shootSpeedRPS) && shooter.hoodAtSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return shooter.leftFlywheelUpToSpeed(shootSpeedRPS) && shooter.rightFlywheelUpToSpeed(shootSpeedRPS) && shooter.hoodAtSetpoint();
    }
}
