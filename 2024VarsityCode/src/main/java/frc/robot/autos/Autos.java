// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  
  // TODO register commands in subsystem constructores using NamedCommands.registerCommand()
  // TODO use AutoBuilder.buildAutoChooser() to choose autos??

    // TODO test of full auto
    public static Command start2ThreePiece(){
        //return new PathPlannerAuto("Start2ThreePiece");
        return Commands.none();
    }
    public static Command startAmp4Piece(){
        return new PathPlannerAuto("FromAmp");
        //return Commands.none();
    }
    public static Command start3Middle(){
        return new PathPlannerAuto("Start3Middle");
        //return Commands.none();
    }
    public static Command start3ThreePiece(){
        return new PathPlannerAuto("Start3ThreePiece");
    }
    public static Command start4ThreePiece(){
        return new PathPlannerAuto("Start4ThreePiece");
        //return Commands.none();
    }
    public static Command start1ThreePiece(){
        //return new PathPlannerAuto("Start1ThreePiece");
        return Commands.none();
    }
    public static Command start2FourPiece(){
        return new PathPlannerAuto("Start2FourPiece");
    }

    // TODO test of a single path
    public static Command testPath(){
        //PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath1");

        return Commands.none();//AutoBuilder.followPath(path);
    }

    // TODO test of a path group
    public static Command testPathGroup(){
        /*
        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("TestAuto");
        Command[] pathGroup = new Command[paths.size()];
        for(int i = 0; i < pathGroup.length; i++){
            pathGroup[i] = AutoBuilder.followPath(paths.get(i));
        }
        */

        return Commands.none();//new SequentialCommandGroup(pathGroup);
    }

    /**
     * Blank Autonomous to be used as default dashboard option
     * @return Autonomous command
     */
    public static Command none(){
        return Commands.none();
    }
}
