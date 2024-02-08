package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmMotorsSubsystem;

public class ArmMotorsCmd extends Command{
    private Supplier<Double> pitchMotor;
    private Double intakeMotorsSpeed, shooterMotorsSpeed, pushMotorSpeed, pitchMotorSpeed;
    private Supplier<Boolean> intakeMotorsRunning, shooterMotorsRunning, pushMotorRunning;
    private ArmMotorsSubsystem armSubsystem;
    private AnalogEncoder encoder;
    public ArmMotorsCmd(ArmMotorsSubsystem armSubsystem, AnalogEncoder encoder, Supplier<Double> pitchMotor, Supplier<Boolean> shooterMotorsRunning, 
        Supplier<Boolean> pushMotorRunning, Supplier<Boolean> intakeMotorsRunning){
        this.pitchMotor = pitchMotor;
        this.shooterMotorsRunning = shooterMotorsRunning;
        this.pushMotorRunning = pushMotorRunning;
        this.intakeMotorsRunning = intakeMotorsRunning;
        this.encoder = encoder;
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
       
    } 
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        pitchMotorSpeed = pitchMotor.get();
        pitchMotorSpeed = pitchMotorSpeed > 0.5 ? 0.5 : pitchMotorSpeed;
        pitchMotorSpeed = pitchMotorSpeed < -0.5 ? -0.5 : pitchMotorSpeed;
        shooterMotorsSpeed = shooterMotorsRunning.get() ? 0.5 : 0;
        pushMotorSpeed = pushMotorRunning.get() ? 0.5 : 0;
        intakeMotorsSpeed = intakeMotorsRunning.get() ? 0.5 : 0;
        if (encoder.getDistance() > 84.9 || encoder.getDistance() < 0.1)
            pitchMotorSpeed = 0.0;
        // if (pitchMotorSpeed > 0.5) pitchMotorSpeed = 0.5;
        // if (pitchMotorSpeed < -0.5) pitchMotorSpeed = -0.5;
        // if (shooterMotorsRunning.get()) shooterMotorsSpeed = 0.5;
        // if (pushMotorRunning.get()) pushMotorSpeed = 0.5;
        // if (intakeMotorsRunning.get()) intakeMotorsSpeed = 0.5;
        armSubsystem.runPitchMotor(pitchMotorSpeed);
        armSubsystem.runShooterMotors(shooterMotorsSpeed);
        armSubsystem.runPushMotor(pushMotorSpeed);
        armSubsystem.runIntakeMotors(intakeMotorsSpeed);
        System.out.println(encoder.getDistance());
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
