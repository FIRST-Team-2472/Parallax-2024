package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class Apriltag_AimingCmd extends Command{
    Supplier<Double> yaw, pitch;
    private double tx, ty, ta, x, y, z;
    private Pose3d tagPos = new Pose3d();
    private final double speed = 0.3;
    LimelightHelpers.LimelightResults llresults;
    public Apriltag_AimingCmd(){
        
    }
}
