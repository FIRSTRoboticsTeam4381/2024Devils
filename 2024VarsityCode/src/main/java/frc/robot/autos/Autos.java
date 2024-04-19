// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
    public static Command start1A1M1M2Red(){ // DONE
        return new PathPlannerAuto("Start1A1M1M2-Red");
    }
    public static Command start2A2M1M2Red(){ // TODO
        return new PathPlannerAuto("Start2A2M1M2-Red");
    }
    public static Command start2A2M2Red(){ // TODO
        return new PathPlannerAuto("Start2A2M2-Red");
    }
    public static Command start2A2M3M2Red(){ // TODO
        return new PathPlannerAuto("Start2A2M3M2-Red");
    }
    public static Command start2A3A2A1M2M1Red(){ // TODO
        return new PathPlannerAuto("Start2A3A2A1M2M1-Red");
    }
    public static Command start3A3M3A2Red(){ // TODO
        return new PathPlannerAuto("Start3A3M3A2-Red");
    }
    public static Command start3M3M2Red(){ // TODO
        return new PathPlannerAuto("Start3M3M2-Red");
    }
    public static Command start4M4M5Red(){ // DONE
        return new PathPlannerAuto("Start4M4M5-Red");
    }

    public static Command start1A1M1M2Blue(){
        return new PathPlannerAuto("Start1A1M1M2-Blue");
    }
    public static Command start2A2M1M2Blue(){
        return new PathPlannerAuto("Start2A2M1M2-Blue");
    }
    public static Command start2A2M2Blue(){
        return new PathPlannerAuto("Start2A2M2-Blue");
    }
    public static Command start2A2M3M2Blue(){
        return new PathPlannerAuto("Start2A2M3M2-Blue");
    }
    public static Command start2A3A2A1M2M1Blue(){
        return new PathPlannerAuto("Start2A3A2A1M2M1-Blue");
    }
    public static Command start3A3M3A2Blue(){
        return new PathPlannerAuto("Start3A3M3A2-Blue");
    }
    public static Command start3M3M2Blue(){
        return new PathPlannerAuto("Start3M3M2-Blue");
    }
    public static Command start4M4M5Blue(){
        return new PathPlannerAuto("Start4M4M5-Blue");
    }
    
    public static Command testAuto1(){
        return new PathPlannerAuto("New Auto1");
    }
    public static Command testAuto2(){
        return new PathPlannerAuto("New Auto2");
    }
    public static Command testAuto3(){
        return new PathPlannerAuto("New Auto3");
    }
    public static Command testAuto4(){
        //return new PathPlannerAuto("New Auto4");
        return Commands.none();
    }
    public static Command destroy(){
        return new PathPlannerAuto("Destroy1");
    }
    public static Command shoot1Taxi(){
        return new PathPlannerAuto("NewWorldsAuto");
    }
    public static Command justShoot(){
        return new PathPlannerAuto("JustShoot");
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
