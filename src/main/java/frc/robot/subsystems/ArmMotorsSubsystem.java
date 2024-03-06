package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.Constants.ArmMotorsConstants.*;
import frc.robot.Constants.SensorConstants;

public class ArmMotorsSubsystem extends SubsystemBase {
    private CANSparkMax pitchMotor = new CANSparkMax(PitchMotor.kPitchMotorId, MotorType.kBrushless);
    private static CANSparkMax shooterTopMotor = new CANSparkMax(ShooterMotors.kTopShooterMotorId, MotorType.kBrushless);
    private CANSparkMax shooterBottomMotor = new CANSparkMax(ShooterMotors.kBottomShooterMotorId, MotorType.kBrushless);
    private CANSparkMax pushMotor = new CANSparkMax(PushMotor.kPushMotorId, MotorType.kBrushless);
    private CANSparkMax intakeTopMotor = new CANSparkMax(IntakeMotors.kTopIntakeMotorId, MotorType.kBrushless);
    private CANSparkMax intakeBottomMotor = new CANSparkMax(IntakeMotors.kBottomIntakeMotorId, MotorType.kBrushless);
    private PIDController pitchPIDController = new PIDController(PitchMotor.kPitchMotorKP, 0, 0);
    DigitalInput photoElectricSensor = new DigitalInput(SensorConstants.kPhotoElectricSensorID);
    public AnalogEncoder pitchMotorEncoder = new AnalogEncoder(ArmMotorsConstants.PitchMotor.kPitchEncoderId);
    ShuffleboardTab encoderTab = Shuffleboard.getTab("Absolute Encoder");
    private GenericEntry internalEncoderPosition;
    private GenericEntry encoderVoltage;
    private GenericEntry encoderDeg;
    private GenericEntry pitchMotorSpeed;
    public double baseIdleForce;

    public ArmMotorsSubsystem() {

        // make sure all of them have the same settings in case we grabbed one with presets
        shooterTopMotor.restoreFactoryDefaults();
        shooterBottomMotor.restoreFactoryDefaults();
        pushMotor.restoreFactoryDefaults();
        intakeTopMotor.restoreFactoryDefaults();
        intakeBottomMotor.restoreFactoryDefaults();
        pitchMotor.restoreFactoryDefaults();

        // sets their constants
        pitchMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        pitchMotor.setSmartCurrentLimit(39);
        
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

    public void runPitchMotor(double motorSpeed, boolean withoutKP) {
        motorSpeed -= 0.15;
        //shuffleboard
        pitchMotorSpeed.setDouble(motorSpeed);
        //running it
        pitchMotor.set(motorSpeed);
    }

    public double getEncoderDeg() {
        return (pitchMotorEncoder.getDistance() + PitchMotor.kPitchEncoderOffset);
    }

    public void runShooterMotors(double motorSpeed) {
        shooterTopMotor.set(-motorSpeed*.9);
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
        runPitchMotor(speed *= 0.1);
    }

    double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public boolean getPhotoElectricSensor(){
        return photoElectricSensor.get();
    }
    
    public double getShooterSpeed(){
        return shooterTopMotor.getEncoder().getVelocity();
    }
}
