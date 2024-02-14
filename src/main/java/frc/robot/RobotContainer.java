package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
  private final String placementone = "Robot 1", placementtwo = "Robot 2", placementthree = "Robot 3", path4 =  "2 in Speaker from Position 2";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CommandSequences commandSequences = new CommandSequences();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();


  XboxController xbox = new XboxController(OperatorConstants.kXboxControllerPort);
  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoyPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoyPort);
  private AnalogEncoder pitchMotorEncoder = new AnalogEncoder(ArmMotorsConstants.PitchMotor.kPitchEncoderId);
  ArmMotorsSubsystem armSubsystem = new ArmMotorsSubsystem(pitchMotorEncoder);
  

  public RobotContainer() {
    armSubsystem.setDefaultCommand(new ArmMotorsCmd(armSubsystem, pitchMotorEncoder, () -> xbox.getLeftY(), // Pitch Motor
      () -> xbox.getLeftTriggerAxis() > 0.5, () -> xbox.getLeftBumper(), // Shooter Motors
        () -> xbox.getRightTriggerAxis() > 0.5, // Push Motor
        () -> xbox.getRightBumper())); // Intake Motors

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
        () -> leftJoystick.getY(),
        () -> -leftJoystick.getX(),
        () -> -rightJoystick.getX(),
        () -> rightJoystick.getRawButton(2)));

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
    ()-> -leftJoystick.getY(),
     ()-> leftJoystick.getX(),
      ()-> rightJoystick.getX(),
       ()-> rightJoystick.getRawButton(1)
    ));
    
    configureBindings();

    m_chooser.addOption(placementone, placementone);
    m_chooser.addOption(placementtwo, placementtwo);
    m_chooser.addOption(placementthree, placementthree);
    m_chooser.addOption(path4, path4);

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  private void configureBindings() {
    new JoystickButton(rightJoystick, 4).onTrue(new InstantCommand(swerveSubsystem :: zeroHeading));
    //new CommandXboxController(OperatorConstants.kXboxControllerPort).a().onTrue(new SetArmPitchCmd(armSubsystem, pitchMotorEncoder, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).b().onTrue(new SetArmPitchCmd(armSubsystem, pitchMotorEncoder, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).x().onTrue(new SetArmPitchCmd(armSubsystem, pitchMotorEncoder, ArmMotorsConstants.PitchMotor.kPitchMotorAmpPresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).y().onTrue(new SetArmPitchCmd(armSubsystem, pitchMotorEncoder, ArmMotorsConstants.PitchMotor.kPitchMotorStandbyPresetAngle));
    if (xbox.getAButton())
      resetEncoder();
  }
  public void resetEncoder(){
    pitchMotorEncoder.reset();
    pitchMotorEncoder.setDistancePerRotation(360);
  }
  public Command getAutonomousCommand() {
    System.out.println("Autos Begun");
       
      m_autoSelected = m_chooser.getSelected();
    /* Replace with actual autos
      if (m_autoSelected == placementone)
      return new ParallelCommandGroup(commandSequences.robot1Command(swerveSubsystem));

    if (m_autoSelected == placementtwo)
      return new ParallelCommandGroup(commandSequences.robot2Command(swerveSubsystem));

    if (m_autoSelected == placementthree)
      return new ParallelCommandGroup(commandSequences.robot3Command(swerveSubsystem));

      if (m_autoSelected == path4)
      return new ParallelCommandGroup(commandSequences.twoInSpeakerPosTwo(swerveSubsystem));*/

    return null;
  }

  public void logSwerve() {

    SmartDashboard.putNumber("Heading", swerveSubsystem.getHeading());
    SmartDashboard.putString("Robot Location", swerveSubsystem.getPose().getTranslation().toString());
    SmartDashboard.putNumber("frontLeft Encoder", swerveSubsystem.getFLAbsEncoder());
    SmartDashboard.putNumber("frontRight Encoder", swerveSubsystem.getFRAbsEncoder());
    SmartDashboard.putNumber("BackLeft Encoder", swerveSubsystem.getBLAbsEncoder());
    SmartDashboard.putNumber("BackRight Encoder", swerveSubsystem.getBRAbsEncoder());
  }

}
