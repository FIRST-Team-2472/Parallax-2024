package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.IntakeLimelightConstants;

public class IntakeDetectorCmd extends Command{
    private double tx, ty, distanceFwd, distanceLR;
    LimelightHelpers.LimelightResults llresults;
    public IntakeDetectorCmd() {
        
        
    }

    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
        tx = LimelightHelpers.getTX("limelight-intake");
        ty = LimelightHelpers.getTY("limelight-intake") + Constants.IntakeLimelightConstants.kIntakeLimelightTYAngleOffset;
        distanceFwd = Constants.IntakeLimelightConstants.kIntakeLimelightHeight / Math.tan(ty);
        distanceLR = distanceFwd * Math.tan(tx);
        
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return false;
    }
}
