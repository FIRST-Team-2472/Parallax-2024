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
    private ArmMotorsSubsystem armSubsystem;
    private PosPose2d posPose2d;
    private Timer timer;
    private boolean shootingInAmp, movedarm, isFinished;
    private double tx;
    public ApriltagAimingCmd(SwerveSubsystem swerveSubsystem, SwerveDriveToPointCmd swerveDriveToPointCmd, ArmMotorsSubsystem armSubsystem, PosPose2d posPose2d){
        this.swerveSubsystem = swerveSubsystem;
        this.swerveDriveToPointCmd = swerveDriveToPointCmd;
        this.armSubsystem = armSubsystem;
        this.posPose2d = posPose2d;
        shootingInAmp = true;
        timer = new Timer();
        addRequirements(armSubsystem);
        addRequirements(swerveSubsystem);
    }
    public ApriltagAimingCmd(SwerveSubsystem swerveSubsystem, SwerveRotateToAngle swerveRotateToAngle, ArmMotorsSubsystem armSubsystem, double tx){//PosPose2d posPose2d){
        this.swerveSubsystem = swerveSubsystem;
        this.swerveRotateToAngle = swerveRotateToAngle;
        this.armSubsystem = armSubsystem;
        this.tx = tx;
        //this.posPose2d = posPose2d;
        shootingInAmp = false;
        timer = new Timer();
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
      addRequirements(armSubsystem);
    }
  
    @Override
    public void execute() {
      if (shootingInAmp) shootInAmp();
      if (!shootingInAmp) shootInSpeaker();
    }

    
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return false;
    }


    public void shootInAmp(){
      
        if(swerveDriveToPointCmd.isFinished()){
        if (!movedarm){
          new SetArmPitchCmd(armSubsystem, PitchMotor.kPitchMotorAmpPresetAngle);
          movedarm = true;
          timer.reset();
        }
        armSubsystem.runShooterMotors(0.5);

        if(timer.hasElapsed(1)){
        
        armSubsystem.runPushMotor(0.5);
        
        if(timer.hasElapsed(3)){

        armSubsystem.runShooterMotors(0.0);
        armSubsystem.runPushMotor(0.0);
        new SetArmPitchCmd(armSubsystem, PitchMotor.kPitchMotorIntakePresetAngle);

        }}}
    }

    public void shootInSpeaker(){
      if(swerveRotateToAngle.isFinished()){
        new SetArmPitchCmd(armSubsystem, PitchMotor.kPitchMotorSpeakerPresetAngle);
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
