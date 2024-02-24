package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmMotorsSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmMotorsConstants.ShooterMotors;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.ArmMotorsSubsystem;

public class ArmMotorsCmd extends Command {
    // Suppliers are used so we can get constant updates to the values
    private Supplier<Double> pitchMotor;
    private Double intakeMotorsSpeed, shooterMotorsSpeed, pushMotorSpeed, pitchMotorSpeed;
    private Supplier<Boolean> intakeMotorsRunning, shooterMotorsSpeaker, shooterMotorsAmp, pushMotorRunning;
    private ArmMotorsSubsystem armSubsystem;

    public ArmMotorsCmd(ArmMotorsSubsystem armSubsystem, Supplier<Double> pitchMotor, Supplier<Boolean> shooterMotorsSpeaker, Supplier<Boolean> shooterMotorsAmp, 
        Supplier<Boolean> pushMotorRunning, Supplier<Boolean> intakeMotorsRunning){
        this.pitchMotor = pitchMotor;
        this.shooterMotorsSpeaker = shooterMotorsSpeaker;
        this.shooterMotorsAmp = shooterMotorsAmp;
        this.pushMotorRunning = pushMotorRunning;
        this.intakeMotorsRunning = intakeMotorsRunning;
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        //this is a child class which inherits some code so we need to call the constructor
        // of the parent class in the 1st line
    }

    @Override
    public void execute() {
        //constantly update the pitch motor input
        pitchMotorSpeed = pitchMotor.get();

        // applies a deadband
        if (pitchMotorSpeed < OIConstants.kArmDeadband && pitchMotorSpeed > -OIConstants.kArmDeadband) pitchMotorSpeed = 0.0;
            pitchMotorSpeed *= 0.3;//slows down the arm
        pitchMotorSpeed = pitchMotorSpeed > 0.1 ? 0.1 : pitchMotorSpeed;
        pitchMotorSpeed = pitchMotorSpeed < -0.1 ? -0.1 : pitchMotorSpeed;
        shooterMotorsSpeed = shooterMotorsSpeaker.get() ? -0.5 : 0;
        shooterMotorsSpeed = shooterMotorsAmp.get() ? -0.25 : 0;
        pushMotorSpeed = pushMotorRunning.get() ? 0.5 : (intakeMotorsRunning.get() && !armSubsystem.getPhotoElectricSensor() ? 0.2 : 0);
        intakeMotorsSpeed = intakeMotorsRunning.get() ? -0.5 : 0;
        // if (pitchMotorSpeed > 0.5) pitchMotorSpeed = 0.5;
        // if (pitchMotorSpeed < -0.5) pitchMotorSpeed = -0.5;
        // if (shooterMotorsRunning.get()) shooterMotorsSpeed = 0.5;
        // if (pushMotorRunning.get()) pushMotorSpeed = 0.5;
        // if (intakeMotorsRunning.get()) intakeMotorsSpeed = 0.5;
        armSubsystem.runPitchMotor(pitchMotorSpeed);

        //runs the shooter motor at 75% speed when we fire in speaker and 50% for the amp
        shooterMotorsSpeed = shooterMotorsSpeaker.get() ? .75 : (shooterMotorsAmp.get() ? 0.5 : 0);
        armSubsystem.runShooterMotors(shooterMotorsSpeed);

        //runs the push motor when ready to fire or during intaking, until it hit the sensor
        pushMotorSpeed = ArmMotorsSubsystem.getShooterSpeed() < -3500 ? 0.5 : (intakeMotorsRunning.get() && !armSubsystem.getPhotoElectricSensor() ? 0.2 : 0);
        armSubsystem.runPushMotor(pushMotorSpeed);

        //runs the intake motors until the sensor is triggered
        intakeMotorsSpeed = intakeMotorsRunning.get() && !armSubsystem.getPhotoElectricSensor() ? 1.0 : 0;
        armSubsystem.runIntakeMotors(intakeMotorsSpeed);
        
        SmartDashboard.putNumber("Shooter speed", ArmMotorsSubsystem.getShooterSpeed());
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
