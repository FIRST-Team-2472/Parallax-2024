package frc.robot.commands;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmMotorsSubsystem;

public class SetArmPitchCmd extends Command {
    private ArmMotorsSubsystem armMotorsSubsystem;
    private final double angleDeg;
    private final Timer timer;
    private AnalogEncoder pitchMotorEncoder;
    
    public SetArmPitchCmd(ArmMotorsSubsystem armMotorsSubsystem, AnalogEncoder pitchMotorEncoder, double angleDeg) {
        this.angleDeg = angleDeg;
        this.timer = new Timer();
        this.armMotorsSubsystem = armMotorsSubsystem;
        this.pitchMotorEncoder = pitchMotorEncoder;
        addRequirements(armMotorsSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        armMotorsSubsystem.runPitchMotorWithKP(angleDeg);
    }

    @Override
    public void end(boolean interrupted) {
        armMotorsSubsystem.runPitchMotor(0.0);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(pitchMotorEncoder.getDistance() - angleDeg) < 1) || (timer.hasElapsed(5));
    }

}
