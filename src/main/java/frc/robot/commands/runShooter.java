package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm_Motors_Subsystem;

public class runShooter extends Command { 

    private Arm_Motors_Subsystem armSubsystem;
    private Timer timer;



    public runShooter(Arm_Motors_Subsystem ArmSubsystem) {
        timer = new Timer();
        this.armSubsystem = ArmSubsystem;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(1)) {
            armSubsystem.runShooterMotors(-1);
        } 

        if (timer.hasElapsed(3)) {
            armSubsystem.runShooterMotors(-1);
            armSubsystem.runPushMotor(2);
        }
    }

    @Override
    public void end(boolean interrupted) {
       armSubsystem.runShooterMotors(0);
       armSubsystem.runPushMotor(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(8);
    }
}
