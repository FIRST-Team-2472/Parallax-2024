package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.Constants.ArmMotorsConstants.*;

public class ArmMotorsSubsystem extends SubsystemBase {
    private CANSparkMax pitchMotor = new CANSparkMax(PitchMotor.kPitchMotorId, MotorType.kBrushless);
    private CANSparkMax shooterTopMotor = new CANSparkMax(ShooterMotors.kTopShooterMotorId, MotorType.kBrushless);
    private CANSparkMax shooterBottomMotor = new CANSparkMax(ShooterMotors.kBottomShooterMotorId, MotorType.kBrushless);
    private CANSparkMax pushMotor = new CANSparkMax(PushMotor.kPushMotorId, MotorType.kBrushless);
    private CANSparkMax intakeTopMotor = new CANSparkMax(IntakeMotors.kTopIntakeMotorId, MotorType.kBrushless);
    private CANSparkMax intakeBottomMotor = new CANSparkMax(IntakeMotors.kBottomIntakeMotorId, MotorType.kBrushless);
    private PIDController pitchPIDController = new PIDController(PitchMotor.kPitchMotorKP, 0, 0);
    public AnalogEncoder pitchMotorEncoder = new AnalogEncoder(ArmMotorsConstants.PitchMotor.kPitchEncoderId);
    ShuffleboardTab encoderTab = Shuffleboard.getTab("Absolute Encoder");
    private GenericEntry internalEncoderPosition;
    private GenericEntry encoderVoltage;
    private GenericEntry encoderDeg;
    private GenericEntry pitchMotorSpeed;
    public double baseIdleForce;

    public ArmMotorsSubsystem() {
        shooterTopMotor.restoreFactoryDefaults();
        shooterBottomMotor.restoreFactoryDefaults();
        pushMotor.restoreFactoryDefaults();
        intakeTopMotor.restoreFactoryDefaults();
        intakeBottomMotor.restoreFactoryDefaults();
        pitchMotor.restoreFactoryDefaults();
        pitchMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        pitchMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) PitchMotor.kPitchEncoderReverseLimit);
        pitchMotor.setSoftLimit(SoftLimitDirection.kForward, (float) PitchMotor.kPitchEncoderForwardLimit);

        pitchMotorEncoder.setDistancePerRotation(360);
        pitchMotor.getEncoder().setPositionConversionFactor(PitchMotor.kPitchInternalEncoderConversionFactor); // -44.44444...
        pitchMotor.getEncoder().setPosition(getEncoderDeg());

        /* Shuffleboard */

        encoderVoltage = encoderTab.add("Encoder Voltage", 0.0d).getEntry();
        encoderDeg = encoderTab.add("Encoder Degrees", 0.0d).getEntry();
        pitchMotorSpeed = encoderTab.add("Pitch Motor Speed", 0.0d).getEntry();
        internalEncoderPosition = encoderTab.add("Internal Encoder Position", 0.0d).getEntry();
    }

    @Override
    public void periodic() {

        // To prevent the arm from falling while idling, we add a base force that
        // prevents the arm from falling, this should always be added to any movement
        // and be clamped to prevent values that are too high. This basically negates
        // gravity.
        baseIdleForce = PitchMotor.kPitchBaseIdleForce
                * Math.sin((getEncoderDeg() / 360) * (2 * Math.PI));

        // If there is any sort of jittering, shaking, or no movement, try running the
        // runPitchMotor() inside here. This will prevent setting the base idle force to
        // the point of not moving, unless it is not moving. periodic() does run before
        // execute, but just in case.
        if (pitchMotor.get() == 0) {
            // runPitchMotor(0);
        }

        // Pass in 0, as runPitchMotor() already adds the baseIdleForce
        runPitchMotor(0);

        /* Shuffleboard */

        // `getAbsolutePosition()` is the *absolute* position of the encoder, no
        // rollovers, no offset.
        encoderVoltage.setDouble(pitchMotorEncoder.getAbsolutePosition());
        // `getDistance()` is the position of the encoder scaled by the distance per
        // rotation, and does have rollovers.
        encoderDeg.setDouble(getEncoderDeg());
        

        internalEncoderPosition.setDouble(pitchMotor.getEncoder().getPosition());
    }

    double addBaseIdleForce(double motorSpeed) {
        //clamps it between -1 and 1
        return clamp(motorSpeed + baseIdleForce, -1.0, 1.0);
    }

    public void runPitchMotor(double motorSpeed) {
        motorSpeed = addBaseIdleForce(motorSpeed);

        // The speed that the speed controller is applying to the motor.
        pitchMotorSpeed.setDouble(motorSpeed);
        pitchMotor.set(motorSpeed);
    }

    public double getEncoderDeg() {
        return (pitchMotorEncoder.getDistance() + PitchMotor.kPitchEncoderOffset);
    }

    public void runShooterMotors(double motorSpeed) {
        shooterTopMotor.set(-motorSpeed);
        shooterBottomMotor.set(motorSpeed);
    }

    public void runPushMotor(double motorSpeed) {
        pushMotor.set(motorSpeed);
    }

    public void resetEncoder(){
        pitchMotorEncoder.reset();
    }

    public void runIntakeMotors(double motorSpeed) {
        intakeTopMotor.set(motorSpeed);
        intakeBottomMotor.set(-motorSpeed);
    }

    public void runPitchMotorWithKP(double angleDeg) {

        double speed = -(pitchPIDController.calculate(getEncoderDeg(), angleDeg));
        runPitchMotor(speed *= 0.3);
    }

    double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
