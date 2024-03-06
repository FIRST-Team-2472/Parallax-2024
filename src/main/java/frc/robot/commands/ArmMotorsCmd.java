package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmMotorsSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmMotorsConstants.ShooterMotors;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.ArmMotorsSubsystem;

public class ArmMotorsCmd extends Command {
    // Suppliers are used so we can get constant updates to the values
    private Supplier<Double> pitchMotor;
    private Double intakeMotorsSpeed, shooterMotorsSpeed, pushMotorSpeed, pitchMotorSpeed;
    private Supplier<Boolean> intakeMotorsRunning, shooterMotorsSpeaker, shooterMotorsAmp;
    private boolean fired, sensed;
    private ArmMotorsSubsystem armSubsystem;
    private Timer timer = new Timer();

    public ArmMotorsCmd(ArmMotorsSubsystem armSubsystem, Supplier<Double> pitchMotor, Supplier<Boolean> shooterMotorsSpeaker, Supplier<Boolean> shooterMotorsAmp, 
         Supplier<Boolean> intakeMotorsRunning){
        this.pitchMotor = pitchMotor;
        fired = false;
        sensed = false;
        this.shooterMotorsSpeaker = shooterMotorsSpeaker;
        this.shooterMotorsAmp = shooterMotorsAmp;
        this.intakeMotorsRunning = intakeMotorsRunning;
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        //this is a child class which inherits some code so we need to call the constructor
        // of the parent class in the 1st line
        timer.start();
    }

    @Override
    public void execute() {
        //constantly update the pitch motor input
        pitchMotorSpeed = pitchMotor.get();

        // applies a deadband
        if (pitchMotorSpeed < OIConstants.kArmDeadband && pitchMotorSpeed > -OIConstants.kArmDeadband) 
            pitchMotorSpeed = 0.0;
        pitchMotorSpeed *= 0.3;//slows down the arm
        armSubsystem.runPitchMotor(pitchMotorSpeed);

        //runs the shooter motor at 75% speed when we fire in speaker and 50% for the amp
        shooterMotorsSpeed = shooterMotorsSpeaker.get() ? .75 : (shooterMotorsAmp.get() ? 0.5 : 0);
        armSubsystem.runShooterMotors(shooterMotorsSpeed);

        if(armSubsystem.getPhotoElectricSensor()){
            sensed = true;
            timer.reset();
        }
        if(armSubsystem.getShooterSpeed() < -3500 || timer.hasElapsed(2)){
            sensed = false;
        }
        

        //runs the push motor when ready to fire or during intaking, until it hit the sensor
        pushMotorSpeed = armSubsystem.getShooterSpeed() < -3500 ? 0.5 : (intakeMotorsRunning.get() && !sensed ? 0.4 : shooterMotorsAmp.get() ? .5 : 0);
        armSubsystem.runPushMotor(pushMotorSpeed);

        System.out.println(armSubsystem.getPhotoElectricSensor());

        //runs the intake motors until the sensor is triggered
        
        intakeMotorsSpeed = ((intakeMotorsRunning.get() && !sensed)|| armSubsystem.getShooterSpeed() < -3500|| shooterMotorsAmp.get()) ? 0.4 : 0;
        armSubsystem.runIntakeMotors(intakeMotorsSpeed);
        
        SmartDashboard.putNumber("Shooter speed", armSubsystem.getShooterSpeed());
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
