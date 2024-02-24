package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorsConstants.PitchMotor;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerveExtras.FieldPose2d;
import frc.robot.subsystems.swerveExtras.PosPose2d;
import frc.robot.Constants.IntakeLimelightConstants;
import frc.robot.commands.ApriltagAimingCmd;
import frc.robot.commands.SetArmPitchCmd;
import frc.robot.commands.SwerveDriveToPointCmd;
import frc.robot.commands.SwerveRotateToAngle;

public class Limelights extends SubsystemBase{
    private Pose3d tagPos = new Pose3d();
    private FieldPose2d fieldpose2d;
    private double tx, ty, distanceFwd, distanceLR;
    private LimelightHelpers.LimelightResults llresults;
    private SwerveSubsystem swerveSubsystem;
    private ArmMotorsSubsystem armSubsystem;
    private SwerveDriveToPointCmd swerveDriveToPointCmd;
    private SwerveRotateToAngle swerveRotateToAngle;
    public Limelights(SwerveSubsystem swerveSubsystem, ArmMotorsSubsystem armSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.armSubsystem = armSubsystem;
    }
      
    public void scanAmpAprilTag(){
      //if (llresults.targetingResults.targets_Fiducials.length > 0) {
        //tagPos = llresults.targetingResults.targets_Fiducials[0].getTargetPose_CameraSpace();
        //s = new FieldPose2d(tagPos.toPose2d().getX(), tagPos.toPose2d().getY(), tagPos.toPose2d().getRotation());
        new ApriltagAimingCmd(swerveSubsystem, swerveDriveToPointCmd, armSubsystem, new PosPose2d(1.84, 7.32, new Rotation2d(-90)));
      //}
    }
    public void scanSpeakerAprilTag(){
        tx = LimelightHelpers.getTX("limelight-shooter");
        new ApriltagAimingCmd(swerveSubsystem, swerveRotateToAngle, armSubsystem, tx);
        /*
        new PosPose2d(0.45, 5.52, new Rotation2d(0));
        new SetArmPitchCmd(armSubsystem, PitchMotor.kPitchMotorAmpPresetAngle);
        timer.restart();
        armSubsystem.runShooterMotors(0.5);
        while(!timer.hasElapsed(1)){}
        armSubsystem.runPushMotor(0.5);
        while(!timer.hasElapsed(3)){}
        armSubsystem.runShooterMotors(0.0);
        armSubsystem.runPushMotor(0.0);
        new SetArmPitchCmd(armSubsystem, PitchMotor.kPitchMotorIntakePresetAngle);
        */
      }
    
     
    public void intakeDetectNote(){
      tx = LimelightHelpers.getTX("limelight-intake");
      ty = LimelightHelpers.getTY("limelight-intake") + IntakeLimelightConstants.kIntakeLimelightTYAngleOffset;
      distanceFwd = IntakeLimelightConstants.kIntakeLimelightHeight / Math.tan(ty);
      distanceLR = distanceFwd * Math.tan(tx);
    }
    
}
