package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerveExtras.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandSequences {

    PosPose2d[] unImportantNodes = new PosPose2d[4];
    PosPose2d[] importantNodes = new PosPose2d[6];
    PosPose2d[] startingNodes = new PosPose2d[5];
    PosPose2d[] collectingNearNodes = new PosPose2d[3];
    PosPose2d[] shootingNearNodes = new PosPose2d[3];
    PosPose2d ampNode = simplePose(1.84, 7.32, -90);

    public CommandSequences() {
        //off side of the stage on loading station side
        unImportantNodes[0] = simplePose(3.9, 1.95, 0);
        //stage side middle note
        unImportantNodes[1] = simplePose(8.24, 0.79, 0);
        //close note 3 for  4 auto
        unImportantNodes[2] = simplePose(2.4, 4.36, 34);
        // amp side note for 3 in auto
        unImportantNodes[3] = simplePose(2.89, 7, -55);

        // non-amp side of Speaker
        importantNodes[0] = simplePose(.55, 4.10, 0);
        // In front of Note
        importantNodes[1] = simplePose(2.2, 4.10, 0);
        // Near front of Speaker
        importantNodes[2] = simplePose(2.2, 5.57, 0);
        // In from of amp
        importantNodes[3] = simplePose(1.84, 7.32, -130);
        // amp side of stage
        importantNodes[4] = simplePose(4.28, 6.30, 0);
        // under the stage
        importantNodes[5] = simplePose(4.78, 4.15, 0);

        // amp start
        startingNodes[0] = simplePose(1.41, 7.26, 0);
        // speaker start 1
        startingNodes[1] = simplePose(0.71, 6.7, 60);
        // speaker start 2
        startingNodes[2] = simplePose(1.4, 5.52, 0);
        // speakr start 3
        startingNodes[3] = simplePose(0.71, 4.38, -60);
        //drive out of starting position 0 - this one is more of a miscellaneous end point
        startingNodes[4] = simplePose(2.46, 7.27, 0);

        // Collecting the near nodes
        collectingNearNodes[0] = simplePose(2.15, 7, 0);
        collectingNearNodes[1] = simplePose(2.15, 5.5, 0); // same as imp. n. [2];
        collectingNearNodes[2] = simplePose(2.15, 4.08, 0); // same as imp. n. [3];

        // Shooting to the speaker from the near nodes
        shootingNearNodes[0] = simplePose(2.9, 7, 30);
        shootingNearNodes[1] = simplePose(2.9, 5.5, 0);
        shootingNearNodes[2] = simplePose(2.9, 4.08, -30);
    }

    public Command driveFromZone(SwerveSubsystem swerveSubsystem) {

        System.out.println("Autos Happening");
        System.out.println(unImportantNodes[0].toString());
        swerveSubsystem.resetOdometry(startingNodes[0]);

        return new SequentialCommandGroup(
                genratePath(swerveSubsystem, startingNodes[0], List.of(), startingNodes[4]));
    }

    public Command twoinampCommand(SwerveSubsystem swerveSubsystem, ArmMotorsSubsystem armSubsystem) {

        System.out.println("Autos Happening");
        System.out.println(unImportantNodes[0].toString());
        swerveSubsystem.resetOdometry(startingNodes[0]);

        return new SequentialCommandGroup(
                new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorAmpPresetAngle),
                genratePath(swerveSubsystem, startingNodes[0], List.of(), importantNodes[3]),
                new runShooter(armSubsystem),
                new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                genratePath(swerveSubsystem, importantNodes[3], List.of(), collectingNearNodes[0]),
                new runIntake(armSubsystem),
                new SwerveRotateToAngle(swerveSubsystem, Rotation2d.fromDegrees(0)));
    }

    public Command twoinspeakerfrompositiontwoCommand(SwerveSubsystem swerveSubsystem,
            ArmMotorsSubsystem armSubsystem) {

        System.out.println("Autos Happening");
        System.out.println(unImportantNodes[0].toString());
        swerveSubsystem.resetOdometry(startingNodes[2]);

        return new SequentialCommandGroup(
                new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),

                new runShooter(armSubsystem),

                new ParallelCommandGroup(
                        new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle,
                                ArmMotorsConstants.PitchMotor.kPitchMotorFarSpeakerPresetAngle, true),

                        genratePath(swerveSubsystem, startingNodes[2], List.of(), importantNodes[2])),

                // new SwerveRotateToAngle(swerveSubsystem, new Rotation2d(20)),
                new runShooter(armSubsystem)

        );
    }

    public Command twoinspeakerfrompositiononeCommand(SwerveSubsystem swerveSubsystem,
            ArmMotorsSubsystem armSubsystem) {
        swerveSubsystem.resetOdometry(startingNodes[1]);

        return new SequentialCommandGroup(
                new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
                new runShooter(armSubsystem),
                new ParallelCommandGroup(
                        new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle),
                        genratePath(swerveSubsystem, startingNodes[1], List.of(), collectingNearNodes[0])),
                new SwerveRotateToAngle(swerveSubsystem, new Rotation2d(5)), // rotation needs to be changed/this is inaccurate
                new runShooter(armSubsystem));
    }

    public Command twoinspeakerfrompositionthreeCommand(SwerveSubsystem swerveSubsystem,
            ArmMotorsSubsystem armSubsystem) {
        swerveSubsystem.resetOdometry(startingNodes[3]);

        return new SequentialCommandGroup(
                new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
                new runShooter(armSubsystem),
                new ParallelCommandGroup(new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle, 
                ArmMotorsConstants.PitchMotor.kPitchMotorFarSpeakerPresetAngle, true), genratePath(swerveSubsystem, startingNodes[3], 
                List.of(), importantNodes[1])),
                new SwerveRotateToAngle(swerveSubsystem, new Rotation2d(5)),
                genratePath(swerveSubsystem, importantNodes[1], List.of(), startingNodes[3]));
    }

    public Command oneinamponefromspeakerpositiononeCommand(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometry(startingNodes[0]);

        return new SequentialCommandGroup(
                genratePath(swerveSubsystem, startingNodes[1], List.of(), collectingNearNodes[0]),
                genratePath(swerveSubsystem, collectingNearNodes[0], List.of(), importantNodes[3]));
    }

    public Command threeinspeakerfrompositionone(SwerveSubsystem swerveSubsystem, ArmMotorsSubsystem armSubsystem) {
        swerveSubsystem.resetOdometry(startingNodes[1]);

        return new SequentialCommandGroup(
                new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
                new runShooter(armSubsystem),
                new ParallelCommandGroup(genratePath(swerveSubsystem, startingNodes[1], List.of(), collectingNearNodes[0]), 
                new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle, ArmMotorsConstants.PitchMotor.kPitchMotorFarSpeakerPresetAngle, true)),
                new runShooter(armSubsystem),
                new ParallelCommandGroup(genratePath(swerveSubsystem, collectingNearNodes[0], List.of(), importantNodes[2]), 
                new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle, ArmMotorsConstants.PitchMotor.kPitchMotorFarSpeakerPresetAngle, true), new runShooter(armSubsystem))
                );
    }

    public Command threeinspeakerfrompositiontwo(SwerveSubsystem swerveSubsystem, ArmMotorsSubsystem armSubsystem) {
        swerveSubsystem.resetOdometry(startingNodes[2]);

        return new SequentialCommandGroup(// fire our note
                new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
                new runShooter(armSubsystem),
                //move, pick up note and move arm to speaker angle
                new ParallelCommandGroup(
                    genratePath(swerveSubsystem, startingNodes[2], List.of(), unImportantNodes[3]),
                    new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle, true)),
                //move back
                genratePath(swerveSubsystem, unImportantNodes[3], List.of(), startingNodes[2]),
                //fire note two
                new runShooter(armSubsystem),
                //move, pick up note and move arm to speaker angle again
                new ParallelCommandGroup(
                    genratePath(swerveSubsystem, startingNodes[2], List.of(), collectingNearNodes[1]),
                    new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle, true)),
                //move back
                genratePath(swerveSubsystem, collectingNearNodes[1], List.of(), startingNodes[2]),
                //fire note 3
                new runShooter(armSubsystem)
                ); 
    }

    public Command fourinspeakerfrompositiontwo(SwerveSubsystem swerveSubsystem, ArmMotorsSubsystem armSubsystem) {
        swerveSubsystem.resetOdometry(startingNodes[2]);

        return new SequentialCommandGroup(// fire our note
                new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
                new runShooter(armSubsystem),
                //move, pick up note and move arm to speaker angle
                new ParallelCommandGroup(
                    genratePath(swerveSubsystem, startingNodes[2], List.of(), unImportantNodes[3]),
                    new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle, true)),
                //move back
                genratePath(swerveSubsystem, unImportantNodes[3], List.of(), startingNodes[2]),
                //fire note two
                new runShooter(armSubsystem),
                //move, pick up note and move arm to speaker angle again
                new ParallelCommandGroup(
                    genratePath(swerveSubsystem, startingNodes[2], List.of(), collectingNearNodes[1]),
                    new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle, true)),
                //move back
                genratePath(swerveSubsystem, collectingNearNodes[1], List.of(), startingNodes[2]),
                //fire note 3
                new runShooter(armSubsystem), //are we really going to even attempt a 4 note auto?
                //move, pick up note and move arm to speaker angle again again
                new ParallelCommandGroup(
                    genratePath(swerveSubsystem, startingNodes[2], List.of(), unImportantNodes[2]),
                    new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle, true)),
                //move back
                genratePath(swerveSubsystem, unImportantNodes[2], List.of(), startingNodes[2]),
                //fire note 4
                new runShooter(armSubsystem)
                );
    }

    public Command underStage(SwerveSubsystem swerveSubsystem) {

        System.out.println("Autos Happening");
        System.out.println(unImportantNodes[0].toString());
        swerveSubsystem.resetOdometry(startingNodes[0]);

        return new SequentialCommandGroup(
                genratePath(swerveSubsystem, startingNodes[0], List.of(importantNodes[4].getPositivePoint()),
                        importantNodes[5]));
    }

    public Command justShoot(ArmMotorsSubsystem armSubsystem) {

        return new SequentialCommandGroup(
                new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
                new runShooter(armSubsystem));
    }

    public Command twoInSpeakerFromPositionthreeFromCenterLine(SwerveSubsystem swerveSubsystem, ArmMotorsSubsystem armSubsystem){
        return new SequentialCommandGroup(
            // angle to shoot
            new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle),
            //fire note one
            new runShooter(armSubsystem),
            //angle down then run intake while driving to the note
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle), 
                    new runIntake(armSubsystem)
                ), 
                genratePath(swerveSubsystem, startingNodes[3], List.of(unImportantNodes[0].getPositivePoint()),unImportantNodes[1])
            ),
            //drive back and angle arm up to speaker
            new ParallelCommandGroup(
                genratePath(swerveSubsystem, unImportantNodes[1], List.of(unImportantNodes[0].getPositivePoint()),startingNodes[3]),
                new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle)
            ),//fire second note
            new runShooter(armSubsystem)
        );
    }

    //the stuff that acually drives the robot around
    // generates a path via points
    private static Command genratePath(SwerveSubsystem swerveSubsystem, PosPose2d startPoint,
            List<PositivePoint> midPoints,
            PosPose2d endPoint) {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        Pose2d driveStartPoint = startPoint.toDrivePose2d();
        Pose2d driveEndPoint = endPoint.toDrivePose2d();
        List<Translation2d> driveMidPoints = new ArrayList<Translation2d>();
        for (int i = 0; i < midPoints.size(); i++)
            driveMidPoints.add(midPoints.get(i).toDrivePos());

        // 2. Generate trajectory
        // Genrates trajectory need to feed start point, a series of inbetween points,
        // and end point
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                driveStartPoint,
                driveMidPoints,
                driveEndPoint,
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                // swerveSubsystm::getPose is same as () -> swerveSubsystem.getPose()
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        // creates a Command list that will reset the Odometry, then move the path, then
        // stop
        return new SequentialCommandGroup(
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    public PosPose2d simplePose(double x, double y, double angleDegrees) {
        return new PosPose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }

}