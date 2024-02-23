package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.swerveExtras.FieldPose2d;

public class ApriltagAimingCmd extends Command{
    Supplier<Double> yaw, pitch;
    private double tx, ty;
    private Pose3d tagPos = new Pose3d();
    private final double speed = 0.3;
    private FieldPose2d fieldpose2d;
    LimelightHelpers.LimelightResults llresults;
    private SwerveSubsystem swerveSubsystem;
    public ApriltagAimingCmd(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
    }
    
    @Override
    public void initialize() {
      
      fieldpose2d = new FieldPose2d(tagPos.toPose2d().getX(), tagPos.toPose2d().getY(), tagPos.toPose2d().getRotation());
    }
  
    @Override
    public void execute() {
      fieldpose2d.toPosPose2d();
      new SwerveDriveToPointCmd(swerveSubsystem, fieldpose2d.toPosPose2d());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return false;
    }
}
