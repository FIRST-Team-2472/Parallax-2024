package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class ApriltagAimingCmd extends Command{
    Supplier<Double> yaw, pitch;
    private double tx, ty;
    private Pose3d tagPos = new Pose3d();
    private final double speed = 0.3;
    LimelightHelpers.LimelightResults llresults;
    public ApriltagAimingCmd(){
        
    }
    
    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return false;
    }
}
