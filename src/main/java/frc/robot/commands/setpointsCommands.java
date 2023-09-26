package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.goalConstants.upperNodeGoal;
import frc.robot.commands.SuperStructure.pivotCommand;
import frc.utils.AutoUtils;

public class setpointsCommands extends AutoUtils{

    public static Command placeConeHigh(){
        return Commands.sequence(Commands.parallel(new pivotCommand(upperNodeGoal.constants)));

    }
    
}
