package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm_Motors_Subsystem;

public class runIntake extends Command { 

    private Arm_Motors_Subsystem armSubsystem;
    private ArmMotorsCmd armCmd;
    private Timer timer;



    public runIntake(Arm_Motors_Subsystem armSubsystem) {
        timer = new Timer();
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        armSubsystem.runIntakeMotors(-0.3);
        armSubsystem.runPushMotor(0.3);
    }

    @Override
    public void end(boolean interrupted) {
       armSubsystem.runIntakeMotors(0);
       armSubsystem.runPushMotor(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }
}
