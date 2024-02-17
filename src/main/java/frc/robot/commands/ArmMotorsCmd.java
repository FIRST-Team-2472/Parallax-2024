package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.ArmMotorsSubsystem;

public class ArmMotorsCmd extends Command{
    // Suppliers are used so we can get constant updates to the values
    private Supplier<Double> pitchMotor;
    private Double intakeMotorsSpeed, shooterMotorsSpeed, pushMotorSpeed, pitchMotorSpeed;
    private Supplier<Boolean> intakeMotorsRunning, shooterMotorsSpeaker, shooterMotorsAmp, pushMotorRunning;
    private ArmMotorsSubsystem armSubsystem;
    private AnalogEncoder pitchMotorEncoder;
    DigitalInput photoElectricSensor = new DigitalInput(SensorConstants.kPhotoElectricSensorID);
    public ArmMotorsCmd(ArmMotorsSubsystem armSubsystem, AnalogEncoder pitchMotorEncoder, Supplier<Double> pitchMotor, Supplier<Boolean> shooterMotorsSpeaker, Supplier<Boolean> shooterMotorsAmp, 
        Supplier<Boolean> pushMotorRunning, Supplier<Boolean> intakeMotorsRunning){
        this.pitchMotor = pitchMotor;
        this.shooterMotorsSpeaker = shooterMotorsSpeaker;
        this.shooterMotorsAmp = shooterMotorsAmp;
        this.pushMotorRunning = pushMotorRunning;
        this.intakeMotorsRunning = intakeMotorsRunning;
        this.armSubsystem = armSubsystem;
        this.pitchMotorEncoder = pitchMotorEncoder;
        // always need to require the subsystem 
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
        armSubsystem.runPitchMotor(pitchMotorSpeed);

        //runs the shooter motor at 75% speed when we fire in speaker and 50% for the amp
        shooterMotorsSpeed = shooterMotorsSpeaker.get() ? .75 : (shooterMotorsAmp.get() ? 0.5 : 0);
        armSubsystem.runShooterMotors(shooterMotorsSpeed);

        //runs the push motor when ready to fire or during intaking, until it hit the sensor
        pushMotorSpeed = pushMotorRunning.get() ? 0.5 : (intakeMotorsRunning.get() && !photoElectricSensor.get() ? 0.2 : 0);
        armSubsystem.runPushMotor(pushMotorSpeed);

        //runs the intake motors until the sensor is triggered
        intakeMotorsSpeed = intakeMotorsRunning.get() && !photoElectricSensor.get() ? 1.0 : 0;
        armSubsystem.runIntakeMotors(intakeMotorsSpeed);
        
        System.out.println(pitchMotorEncoder.getDistance());
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
