package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PnuematicsSubsystem;

public class PnuematicsCmd extends Command{

    PnuematicsSubsystem pnuematicsSubsystem;

    public PnuematicsCmd(PnuematicsSubsystem pnuematicsSubsystem){
        this.pnuematicsSubsystem = pnuematicsSubsystem;
        addRequirements(pnuematicsSubsystem);
    }
}
