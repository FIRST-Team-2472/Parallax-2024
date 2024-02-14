package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.TargetPosConstants;
import frc.robot.subsystems.swerveExtras.AccelerationLimiter;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(//
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetAng,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed );

    private final SwerveModule frontRight = new SwerveModule(//
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetAng,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed );

    private final SwerveModule backLeft = new SwerveModule(//
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetAng,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed );

    private final SwerveModule backRight = new SwerveModule(//
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetAng,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed );

    private final AccelerationLimiter xLimiter, yLimiter, turningLimiter;
    private PIDController xController, yController, thetaController;

    private final Pigeon2 gyro = new Pigeon2(SensorConstants.kPigeonID);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getModulePositions());

    private static BooleanSubscriber isOnRed;
    private static final SendableChooser<String> colorChooser = new SendableChooser<>();
    private final String red = "Red", blue = "Blue";

    public SwerveSubsystem(){

        // Gets tabs from Shuffleboard
        ShuffleboardTab programmerBoard = Shuffleboard.getTab("Programmer Board");
        ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");

        // Gets the field infomation
        NetworkTable firstInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");
        // Gets the team color from the field information
        isOnRed = firstInfo.getBooleanTopic("IsRedAlliance").subscribe(false);

        // makes a team color choser
        colorChooser.addOption(red, red);
        colorChooser.addOption(blue, blue);
        driverBoard.add("Team Chooser", colorChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

        xLimiter = new AccelerationLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter = new AccelerationLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        turningLimiter = new AccelerationLimiter(DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);

        xController = new PIDController(TargetPosConstants.kPDriveController, 0, 0);
        yController = new PIDController(TargetPosConstants.kPDriveController, 0, 0);
        thetaController = new PIDController(TargetPosConstants.kPAngleController, 0, 0);

        new Thread(() -> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            }catch(Exception e){

            }
        }).start();
    }

    public void zeroHeading(){
        gyro.setYaw(0);
    }

    public double getHeading(){
        return -gyro.getYaw().getValue();
    }
    
    public boolean isAtPoint(Translation2d targetDrivePos) {
        return getPose().getTranslation().getDistance(targetDrivePos) //
                <= TargetPosConstants.kAcceptableDistanceError; //
    }

    public boolean isAtAngle(Rotation2d angle) {
        return Math.abs(getRotation2d().minus(angle).getDegrees()) //
                <= TargetPosConstants.kAcceptableAngleError;
    }

    // gets our current velocity relative to the y of the field
    public double getYSpeedFieldRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.vyMetersPerSecond;
    }

    public double getXSpeedFieldRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.vxMetersPerSecond;
    }

    // gets our current velocity relative to the x of the robot (front/back)
    public double getXSpeedRobotRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());

        return temp.vxMetersPerSecond;
    }

    // gets our current velocity relative to the y of the robot (left/right)
    public double getYSpeedRobotRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());

        return temp.vyMetersPerSecond;
    }

    // gets our current angular velocity
    public double getRotationalSpeed() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.omegaRadiansPerSecond;
    }
    
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }
    
    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }
    public void intializeJoystickRunFromField() {
        xLimiter.setLimit(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter.setLimit(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        turningLimiter.setLimit(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        xLimiter.reset(getXSpeedFieldRel());
        yLimiter.reset(getYSpeedFieldRel());
        turningLimiter.reset(getRotationalSpeed());
    }

    public void excuteJoystickRunFromField(double xSpeedPercent, double ySpeedPercent, double thetaSpeedPercent) {
        // 3. Make the driving smoother (limits acceleration)
        xSpeedPercent = xLimiter.calculate(xSpeedPercent * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        ySpeedPercent = yLimiter.calculate(ySpeedPercent * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        thetaSpeedPercent = turningLimiter
                .calculate(thetaSpeedPercent * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);

        runModulesFieldRelative(xSpeedPercent, ySpeedPercent, thetaSpeedPercent);
    }

    public void initializeDriveToPointAndRotate() {
        xLimiter.setLimit(TargetPosConstants.kForwardMaxAcceleration,
        TargetPosConstants.kBackwardMaxAcceleration);
        yLimiter.setLimit(TargetPosConstants.kForwardMaxAcceleration,
        TargetPosConstants.kBackwardMaxAcceleration);
        xLimiter.reset(getXSpeedFieldRel());
        yLimiter.reset(getYSpeedFieldRel());

        xController.setPID(TargetPosConstants.kPDriveController, 0, 0);
        xController.reset();
        yController.setPID(TargetPosConstants.kPDriveController, 0, 0);
        yController.reset();
        thetaController.setPID(TargetPosConstants.kPAngleController, 0, 0);
        thetaController.reset();
    }
    public void excuteDriveToPointAndRotate(Pose2d targetPosition) {
        double xSpeed = MathUtil.clamp(
                xController.calculate(getPose().getX(), targetPosition.getX()), -1, 1);
        double ySpeed = MathUtil.clamp(
                yController.calculate(getPose().getY(), targetPosition.getY()), -1, 1);

        Rotation2d angleDifference = getRotation2d().minus(targetPosition.getRotation());
        double turningSpeed = MathUtil.clamp(thetaController.calculate(angleDifference.getRadians(),
                0), -1, 1);
        turningSpeed *= TargetPosConstants.kMaxAngularSpeed;
        turningSpeed += Math.copySign(TargetPosConstants.kMinAngluarSpeedRadians, turningSpeed);

        xSpeed = xLimiter.calculate(xSpeed * TargetPosConstants.kMaxSpeedMetersPerSecond);
        ySpeed = yLimiter.calculate(ySpeed * TargetPosConstants.kMaxSpeedMetersPerSecond);

        double unitCircleAngle = Math.atan2(ySpeed, xSpeed);
        xSpeed += Math.copySign(TargetPosConstants.kMinSpeedMetersPerSec, xSpeed) * Math.abs(Math.cos(unitCircleAngle));
        ySpeed += Math.copySign(TargetPosConstants.kMinSpeedMetersPerSec, ySpeed) * Math.abs(Math.sin(unitCircleAngle));

        runModulesFieldRelative(xSpeed, ySpeed, turningSpeed);
    }
    public void initializeRotateToAngle() {
        thetaController.setPID(TargetPosConstants.kPAngleController, 0, 0);
        thetaController.reset();
    }

    public void excuteRotateToAngle(Rotation2d targetPosition) {
        Rotation2d angleDifference = getRotation2d().minus(targetPosition);
        double turningSpeed = MathUtil.clamp(thetaController.calculate(angleDifference.getRadians(),
                0), -1, 1) * TargetPosConstants.kMaxAngularSpeed;

        turningSpeed += Math.copySign(TargetPosConstants.kMinAngluarSpeedRadians, turningSpeed);

        runModulesFieldRelative(0, 0, turningSpeed);
    }
    private void runModulesFieldRelative(double xSpeed, double ySpeed, double turningSpeed) {
        // Converts robot speeds to speeds relative to field
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, getRotation2d());

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Output each module states to wheels
        setModuleStates(moduleStates);
    }

    @Override
    public void periodic(){
        odometer.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("frontLeft Encoder", getFLAbsEncoder());
        SmartDashboard.putNumber("frontRight Encoder", getFRAbsEncoder());
        SmartDashboard.putNumber("BackLeft Encoder", getBLAbsEncoder());
        SmartDashboard.putNumber("BackRight Encoder", getBRAbsEncoder());
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

        logSwerveDesiredStates(desiredStates);
    }
    
    public SwerveModulePosition[] getModulePositions() {
        // Finds the position of each individual module based on the encoder values.
        SwerveModulePosition[] temp = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        return temp;
    }

    // used for anything that requires team color.
    // this is housed in swerve subsystem since it uses it the most
    public static boolean isOnRed() {
        // gets the selected team color from the suffleboard
        String choices = colorChooser.getSelected();
        if (choices == "Red")
            return true;
        if (choices == "Blue")
            return false;
        // if no team selected on suffleboard, it will default to the field info
        return isOnRed.get();
    }

    // send over shuffleboard values
    public double getFLAbsEncoder(){
        return frontLeft.getUnfilteredPosition();
    }
    public double getFRAbsEncoder(){
        return frontRight.getUnfilteredPosition();
    }
    public double getBLAbsEncoder(){
        return backLeft.getUnfilteredPosition();
    }
    public double getBRAbsEncoder(){
        return backRight.getUnfilteredPosition();
    }

    /* -- LOGGING -- */
    /*
     * Logging inputs and other random values with Advantage Kit
     * is extremely simple. Literally just run:
     * 
     * Logger.recordOutput(<name>, <value>);
     * 
     * <name> is a string containing the location were the value will be
     * saved (and thus appear in Advantage Scope). Such as, "Odometry/Location" will
     * save the value as "Location" under the "Odometry" Folder.
     * 
     * <value> can be of any rudimentary variable, Translation2d, Pose3d,
     * SwerveModuleState, and Mechanism2d. These are the values that will be logged.
     * All supported variable types are listed here:
     * https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/DATA-FLOW.md#data-types
     * 
     * Logging can also be done using the @AutoLogOutput annotation.
     * More info here: 
     * https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/RECORDING-OUTPUTS.md#autologoutput-annotation
     * 
     * 
     */

    // Send the swerve modules' encoder positions to Advantage Kit
    public void logSwerveStates() {

        Logger.recordOutput("SwerveState", new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        });
    }

    // Send Swerve Desired Rotation and speed to Advantage Kit
    public void logSwerveDesiredStates(SwerveModuleState[] desiredStates) {
        Logger.recordOutput("DesiredSwerveState", desiredStates);
    }

    // Send Pigeon Rotation to Advantage Kit (useful for seeing robot rotation)
    public void logPigeonState() {
        Logger.recordOutput("PigeonGyro", gyro.getRotation2d());
    }

    // Send Odometry Position on field to Advantage Kit
    public void logOdometry() {
        Logger.recordOutput("Odometry/Location", odometer.getPoseMeters());
    }
}
