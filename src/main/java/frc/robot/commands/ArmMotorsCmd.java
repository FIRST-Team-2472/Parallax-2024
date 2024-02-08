package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm_Motors_Subsystem;

public class ArmMotorsCmd extends Command{
    private Supplier<Double> pitchMotor;
    private Double intakeMotorsSpeed, shooterMotorsSpeed, pushMotorSpeed, pitchMotorSpeed;
    private Supplier<Boolean> intakeMotorsRunning, shooterMotorsRunning, pushMotorRunning;
    private Arm_Motors_Subsystem armSubsystem;
    public ArmMotorsCmd(Arm_Motors_Subsystem armSubsystem, Supplier<Double> pitchMotor, Supplier<Boolean> shooterMotorsRunning, 
        Supplier<Boolean> pushMotorRunning, Supplier<Boolean> intakeMotorsRunning){
        this.pitchMotor = pitchMotor;
        this.shooterMotorsRunning = shooterMotorsRunning;
        this.pushMotorRunning = pushMotorRunning;
        this.intakeMotorsRunning = intakeMotorsRunning;
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
        pitchMotorSpeed = pitchMotorSpeed > 0.1 ? 0.1 : pitchMotorSpeed;
        pitchMotorSpeed = pitchMotorSpeed < -0.1 ? -0.1 : pitchMotorSpeed;
        shooterMotorsSpeed = shooterMotorsRunning.get() ? 0.5 : 0;
        pushMotorSpeed = pushMotorRunning.get() ? 0.5 : 0;
        intakeMotorsSpeed = intakeMotorsRunning.get() ? 0.5 : 0;
        // if (pitchMotorSpeed > 0.5) pitchMotorSpeed = 0.5;
        // if (pitchMotorSpeed < -0.5) pitchMotorSpeed = -0.5;
        // if (shooterMotorsRunning.get()) shooterMotorsSpeed = 0.5;
        // if (pushMotorRunning.get()) pushMotorSpeed = 0.5;
        // if (intakeMotorsRunning.get()) intakeMotorsSpeed = 0.5;
        armSubsystem.runPitchMotor(pitchMotorSpeed);
        armSubsystem.runShooterMotors(shooterMotorsSpeed);
        armSubsystem.runPushMotor(pushMotorSpeed);
        armSubsystem.runIntakeMotors(intakeMotorsSpeed);
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
