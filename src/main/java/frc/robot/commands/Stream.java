package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class Stream extends Command {
    LimelightHelpers.LimelightResults llresults;

    public Stream() {
        initStreamShuffleBoard();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
    }

    void initStreamShuffleBoard() {
        System.out.println("LimeLight Shuffleboard Starting");
        ShuffleboardTab limelightBoard = Shuffleboard.getTab("Stream");
        limelightBoard.addCamera("LimeLightShooter Stream", "limelight_shooter", "mjpg:http://limelight-shooter.local:5800").withSize(4, 4);
        limelightBoard.addCamera("LimeLightIntake Stream", "limelight_intake", "mjpg:http://limelight-intake.local:5800").withSize(4, 4);
        Shuffleboard.update();
    }
}
