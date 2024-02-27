package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmMotorsConstants.PitchMotor;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ArmMotorsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.swerveExtras.PosPose2d;

public class ApriltagAimingCmd extends Command{
    private SwerveSubsystem swerveSubsystem;
    private SwerveDriveToPointCmd swerveDriveToPointCmd;
    private SwerveRotateToAngle swerveRotateToAngle;
    private SetArmPitchCmd setArmPitchCmd;
    private ArmMotorsSubsystem armSubsystem;
    private PosPose2d posPose2d;
    private Timer timer;
    private boolean shootingInAmp, movedarm, isCompleted;
    private double tx;
    public ApriltagAimingCmd(SwerveSubsystem swerveSubsystem, SwerveDriveToPointCmd swerveDriveToPointCmd, ArmMotorsSubsystem armSubsystem, SetArmPitchCmd setArmPitchCmd, PosPose2d posPose2d){
        this.swerveSubsystem = swerveSubsystem;
        this.swerveDriveToPointCmd = swerveDriveToPointCmd;
        this.armSubsystem = armSubsystem;
        this.setArmPitchCmd = setArmPitchCmd;
        this.posPose2d = posPose2d;
        shootingInAmp = true;
        addRequirements(armSubsystem);
        addRequirements(swerveSubsystem);
    }
    public ApriltagAimingCmd(SwerveSubsystem swerveSubsystem, SwerveRotateToAngle swerveRotateToAngle, ArmMotorsSubsystem armSubsystem, SetArmPitchCmd setArmPitchCmd, double tx){//PosPose2d posPose2d){
        this.swerveSubsystem = swerveSubsystem;
        this.swerveRotateToAngle = swerveRotateToAngle;
        this.armSubsystem = armSubsystem;
        this.setArmPitchCmd = setArmPitchCmd;
        this.tx = tx;
        //this.posPose2d = posPose2d;
        shootingInAmp = false;
        addRequirements(armSubsystem);
        addRequirements(swerveSubsystem);
    }
    @Override
    public void initialize() {
      if (shootingInAmp)
        swerveDriveToPointCmd = new SwerveDriveToPointCmd(swerveSubsystem, posPose2d);
      else
        swerveRotateToAngle = new SwerveRotateToAngle(swerveSubsystem, (new Rotation2d(tx).plus(swerveSubsystem.getRotation2d())));
      movedarm = false;
      isCompleted = false;
    }
  
    @Override
    public void execute() {
      if (shootingInAmp) shootInAmp();
      else shootInSpeaker();
    }

    
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return if(isCompleted || timer.hasElapsed(5));
    }


    public void shootInAmp(){
      if(swerveDriveToPointCmd.isFinished()){
        if (!movedarm){
          setArmPitchCmd = new setArmPitchCmd(armSubsystem, PitchMotor.kPitchMotorAmpPresetAngle);
          movedarm = true;
          timer = new Timer();
        }
        armSubsystem.runShooterMotors(0.5);
        if(timer.hasElapsed(1) && setArmPitchCmd.isFinished()){
          armSubsystem.runPushMotor(0.5);
          if(timer.hasElapsed(3)){
            armSubsystem.runShooterMotors(0.0);
            armSubsystem.runPushMotor(0.0);
            setArmPitchCmd = new setArmPitchCmd(armSubsystem, PitchMotor.kPitchMotorIntakePresetAngle);
            if (setArmPitchCmd.isFinished())
              isCompleted = true;
          }
        }
      }
    }

    public void shootInSpeaker(){
      if(swerveRotateToAngle.isFinished()){
        if (!movedarm){
        setArmPitchCmd = new setArmPitchCmd(armSubsystem, PitchMotor.kPitchMotorSpeakerPresetAngle);
        movedarm = true;
        timer = new Timer();
        }
        if (setArmPitchCmd.isFinished())
          isCompleted = true;
      }
      /*
      new SetArmPitchCmd(armSubsystem, PitchMotor.kPitchMotorSpeakerPresetAngle);
      armSubsystem.runShooterMotors(.75);

        if(timer.hasElapsed(1)){
        
        armSubsystem.runPushMotor(0.5);
        
        if(timer.hasElapsed(3)){

        armSubsystem.runShooterMotors(0.0);
        armSubsystem.runPushMotor(0.0);
        new SetArmPitchCmd(armSubsystem, PitchMotor.kPitchMotorIntakePresetAngle);
        */
    }//}}
}
