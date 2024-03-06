package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.subsystems.ArmMotorsSubsystem;

public class SetArmPitchCmd extends Command {
    private ArmMotorsSubsystem armMotorsSubsystem;
    private double angleDeg;
    private final Timer timer, seentTimer;
    private int rev;
    private boolean seen;
    
    public SetArmPitchCmd(ArmMotorsSubsystem armMotorsSubsystem, double angleDeg) {
        this.angleDeg = angleDeg;
        this.timer = new Timer();
        seentTimer = new Timer();
        this.armMotorsSubsystem = armMotorsSubsystem;
        addRequirements(armMotorsSubsystem);
    }
    public SetArmPitchCmd(ArmMotorsSubsystem armMotorsSubsystem, double angleDeg, int rev) {
        this(armMotorsSubsystem, angleDeg);
        this.rev = rev;
        seen = false;
        addRequirements(armMotorsSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        if(armMotorsSubsystem.getPhotoElectricSensor() && !seen && rev == 2){
            seen = true;
            seentTimer.start();
            angleDeg = ArmMotorsConstants.PitchMotor.kPitchMotorFarSpeakerPresetAngle;
        }
        armMotorsSubsystem.runPitchMotorWithKP(angleDeg);
        if(rev == 1)
            armMotorsSubsystem.runShooterMotors(.7);

        if (rev == 2 && !seen) {
            armMotorsSubsystem.runIntakeMotors(0.6);
            armMotorsSubsystem.runPushMotor(0.6);
            armMotorsSubsystem.runShooterMotors(.7);
        }
        if(seen && !seentTimer.hasElapsed(0.2)) {
            armMotorsSubsystem.runPushMotor(-0.3);
        }
        if(seen && seentTimer.hasElapsed(0.2)){
            armMotorsSubsystem.runShooterMotors(.7);
        }
    } 

    @Override
    public void end(boolean interrupted) {
        armMotorsSubsystem.runPitchMotor(0.0);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(armMotorsSubsystem.getEncoderDeg() - angleDeg) < 0.5) || (timer.hasElapsed(3));
    }

}
