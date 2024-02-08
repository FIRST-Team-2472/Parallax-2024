package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
  Arm_Motors_Subsystem armSubsystem = new Arm_Motors_Subsystem();
  XboxController xbox = new XboxController(OperatorConstants.kDriverControllerPort);
  public RobotContainer() {
    armSubsystem.setDefaultCommand(new ArmMotorsCmd(armSubsystem, () -> xbox.getLeftY(), // Pitch Motor
      () -> xbox.getLeftTriggerAxis() > 0.5, () -> xbox.getLeftBumper(), // Shooter Motors
        () -> xbox.getRightTriggerAxis() > 0.5, // Push Motor
          () -> xbox.getRightBumper())); // Intake Motors

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
