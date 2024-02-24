package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.CommandSequences;

public class RobotContainer {
  private final String placementone = "2 in amp command", placementtwo = "2 in speaker from position 2", placementthree = "2 in speaker from position 1", path4 =  "2 in Speaker from Position 3", testingPath =  "Test Path";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CommandSequences commandSequences = new CommandSequences();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final PnuematicsSubsystem pnuematicsSubsystem = new PnuematicsSubsystem();


  private final ArmMotorsSubsystem armSubsystem = new ArmMotorsSubsystem();

  XboxController xbox = new XboxController(OperatorConstants.kXboxControllerPort);
  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoyPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoyPort);
  
  

  public RobotContainer() {
/*    armSubsystem.setDefaultCommand(new ArmMotorsCmd(armSubsystem, () -> xbox.getLeftY())); // Pitch Motor
       () -> xbox.getLeftTriggerAxis() > 0.5, () -> xbox.getLeftBumper(), // Shooter Motors
      () -> leftJoystick.getRawButton(1))); // Intake Motors
        () -> xbox.getRightTriggerAxis() > 0.5, // Push Motor
          () -> xbox.getRightBumper())); // Intake Motors */

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
      ()-> -leftJoystick.getY(),
      ()-> leftJoystick.getX(),
      ()-> rightJoystick.getX(),
      ()-> rightJoystick.getRawButton(1)
    ));

    pnuematicsSubsystem.setDefaultCommand(new PnuematicsCmd(pnuematicsSubsystem));
    
    configureBindings();

    m_chooser.addOption(placementone, placementone);
    m_chooser.addOption(placementtwo, placementtwo);
    m_chooser.addOption(placementthree, placementthree);
    m_chooser.addOption(path4, path4);
    m_chooser.addOption(testingPath, testingPath);

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  private void configureBindings() {
    new JoystickButton(rightJoystick, 4).onTrue(new InstantCommand(swerveSubsystem :: zeroHeading));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).axisGreaterThan(1, .5).whileTrue(new runShooter(armSubsystem));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).leftTrigger(0.5).whileTrue(new runShooter(armSubsystem));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).leftBumper().onTrue(new runShooter(armSubsystem));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).rightBumper().onTrue(new runIntake(armSubsystem));

    new CommandXboxController(OperatorConstants.kXboxControllerPort).pov(180).onTrue(new InstantCommand(pnuematicsSubsystem :: toggleSmallpnuematics));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).pov(0).onTrue(new InstantCommand(pnuematicsSubsystem :: toggleBigpnuematics));

    new CommandXboxController(OperatorConstants.kXboxControllerPort).a().onTrue(new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).b().onTrue(new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).x().onTrue(new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorAmpPresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).y().onTrue(new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorStandbyPresetAngle));
  }

  public Command getAutonomousCommand() {
    System.out.println("Autos Begun");
       
      m_autoSelected = m_chooser.getSelected();

      if (m_autoSelected == testingPath)
        return new ParallelCommandGroup(commandSequences.testingPath(swerveSubsystem));


      if (m_autoSelected == placementone)
      return new ParallelCommandGroup(commandSequences.twoinampCommand(swerveSubsystem, armSubsystem));

      if (m_autoSelected == placementtwo)
      return new ParallelCommandGroup(commandSequences.twoinspeakerfrompositiontwoCommand(swerveSubsystem));

      if (m_autoSelected == placementthree)
      return new ParallelCommandGroup(commandSequences.twoinspeakerfrompositiononeCommand(swerveSubsystem));

      if (m_autoSelected == path4)
      return new ParallelCommandGroup(commandSequences.twoinspeakerfrompositionthreeCommand(swerveSubsystem));

    return null;
  }

  public void logSwerve() {

    SmartDashboard.putNumber("Heading", swerveSubsystem.getHeading());
    SmartDashboard.putString("Robot Location", swerveSubsystem.getPose().getTranslation().toString());
    SmartDashboard.putNumber("frontLeft Encoder", swerveSubsystem.getFLAbsEncoder());
    SmartDashboard.putNumber("frontRight Encoder", swerveSubsystem.getFRAbsEncoder());
    SmartDashboard.putNumber("BackLeft Encoder", swerveSubsystem.getBLAbsEncoder());
    SmartDashboard.putNumber("BackRight Encoder", swerveSubsystem.getBRAbsEncoder());
    SmartDashboard.putNumber("Shooter speed", ArmMotorsSubsystem.getShooterSpeed());
    //SmartDashboard.putNumber("Arm Encoder", armSubsystem.getAbsoluteEncoder());
  }

}
