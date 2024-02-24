package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> slowed;
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
    Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> slowed) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.slowed = slowed;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        swerveSubsystem.intializeJoystickRunFromField();
    }

    @Override
    public void execute() {
        // get joystick values
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get() / 2;

        // deadband (area that doesnt actually result in an input)
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? (!slowed.get() ? xSpeed : xSpeed * OperatorConstants.kSlowedSpeed) : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? (!slowed.get() ? ySpeed : ySpeed * OperatorConstants.kSlowedSpeed) : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        swerveSubsystem.excuteJoystickRunFromField(xSpeed, ySpeed, turningSpeed);


    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
