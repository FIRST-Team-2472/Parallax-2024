package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmMotorsSubsystem;

public class runIntake extends Command { 

    private ArmMotorsSubsystem armSubsystem;
    private ArmMotorsCmd armCmd;
    private Timer timer, seenTimer;
    private boolean seen;


    public runIntake(ArmMotorsSubsystem armSubsystem) {
        timer = new Timer();
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        if(!seen){
            armSubsystem.runIntakeMotors(0.6);
            armSubsystem.runPushMotor(0.6);
        }else if(!seenTimer.hasElapsed(0.2)){
            armSubsystem.runIntakeMotors(-0.3);
            armSubsystem.runPushMotor(-0.3);
        }
        if(armSubsystem.getPhotoElectricSensor() && !seen){
            seen = true;
            seenTimer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
       armSubsystem.runIntakeMotors(0);
       armSubsystem.runPushMotor(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(5) || seenTimer.hasElapsed(0.2);
    }
}
