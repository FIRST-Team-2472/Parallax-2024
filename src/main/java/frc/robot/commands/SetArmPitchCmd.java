package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.subsystems.ArmMotorsSubsystem;

public class SetArmPitchCmd extends Command {
    private ArmMotorsSubsystem armMotorsSubsystem;
    private double angleDeg, secondAngleDeg;
    private final Timer timer, seentTimer;
    private boolean seen, collection;
    
    public SetArmPitchCmd(ArmMotorsSubsystem armMotorsSubsystem, double angleDeg) {
        this.angleDeg = angleDeg;
        this.timer = new Timer();
        seentTimer = new Timer();
        this.armMotorsSubsystem = armMotorsSubsystem;
        addRequirements(armMotorsSubsystem);
    }
    public SetArmPitchCmd(ArmMotorsSubsystem armMotorsSubsystem, double angleDeg, double secondAngleDeg, boolean collection) {
        this(armMotorsSubsystem, angleDeg);
        this.secondAngleDeg = secondAngleDeg;
        this.collection = collection;
        seen = false;
        addRequirements(armMotorsSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        //once we see the note, we reverse the intake motors for a split second 
        //to counteract the inertia and then we start switching to the second
        //angle we desired
        if(collection && armMotorsSubsystem.getPhotoElectricSensor() && !seen){
            seen = true;
            seentTimer.start();
            angleDeg = secondAngleDeg;
        }
        //just angling our arm
        armMotorsSubsystem.runPitchMotorWithKP(angleDeg);

        //just revving up the motors so we dont waste time
            armMotorsSubsystem.runShooterMotors(.7);

        //if we plan to collect a note and havent seen it we run the intake system
        // and rev the motors
        if (collection && !seen) {
            armMotorsSubsystem.runIntakeMotors(0.6);
            armMotorsSubsystem.runPushMotor(0.6);
            armMotorsSubsystem.runShooterMotors(.7);
        }

        //basically run it in reverse for .2 seconds after to remove any intertia
        if(seen && !seentTimer.hasElapsed(0.2)) {
            armMotorsSubsystem.runPushMotor(-0.3);
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
