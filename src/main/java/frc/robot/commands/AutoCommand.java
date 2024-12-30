// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;
import java.util.HashMap;

/** Add your docs here. */
public class AutoCommand {
    Command autoCommand;
    public String name;
    private String[] potentialContinuations;   
    private static HashMap<String, AutoCommand> pathKeys = new HashMap<String, AutoCommand>();
    private static ArrayList<AutoCommand> autoList2 = new ArrayList<AutoCommand>();
    //if you have questions ask ethan
    public static void loadAutos(){
        pathKeys.put("Test1", Test1());
        pathKeys.put("Test2", Test2());
        pathKeys.put("Test3", Test3());
        pathKeys.put("Test4", Test4());
        pathKeys.put("meterForwardTest", meterForwardTest());
    }

    public AutoCommand(String name, Command autoCommand, String[] potentialContinuations){
        this.name = name;
        this.autoCommand = autoCommand;
        this.potentialContinuations = potentialContinuations;
    }

    public static void clearContinuations(){
        autoList2.clear();
    }

    public static void fillAutosList(AutoCommand auto){
        for(String name : auto.potentialContinuations){
            autoList2.add(getAuto(name));
        }
    }

    public Command getCommand(){
        return this.autoCommand;
    }



    public static ArrayList<AutoCommand> getPotentialContinuations() {
        return autoList2;
    }

    public static AutoCommand Test1(){
        return new AutoCommand("Test1", Autos.Test1(), new String[]{"Test2", "Test3", "Test4", "meterForwardTest"});
    }

    public static AutoCommand Test2(){
        return new AutoCommand("Test2", Autos.Test2(), new String[]{"Test1", "Test3", "Test4"});
    }

    public static AutoCommand Test3() {
        return new AutoCommand("Test3", Autos.Test3(), new String[]{"Test1", "Test2", "Test4"});
    }

    public static AutoCommand Test4() {
        return new AutoCommand("Test4", Autos.Test4(), new String[]{"Test1", "Test2", "Test3"});
    }

    public static AutoCommand meterForwardTest() {
        return new AutoCommand("meterForwardTest", Autos.meterForwardTest(), new String[]{"Test1", "Test2", "Test3", "Test4"});
    }

    public static AutoCommand getAuto(String name){
        return pathKeys.get(name);
    }
}
