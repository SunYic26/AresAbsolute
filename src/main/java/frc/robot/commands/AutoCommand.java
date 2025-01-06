// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;

// import java.util.ArrayList;
// import java.util.HashMap;

// /** Add your docs here. */
// public class AutoCommand {
//     Command autoCommand;
//     public String name;
//     private String[] potentialContinuations;   
//     private static HashMap<String, AutoCommand> pathKeys = new HashMap<String, AutoCommand>();
//     private static ArrayList<AutoCommand> autoList2 = new ArrayList<AutoCommand>();
//     public static void loadAutos(){
//         pathKeys.put("Test1", Test1());
//         pathKeys.put("Test2", Test2());
//     }
//     public AutoCommand(String name, Command autoCommand, String[] potentialContinuations){
//         this.name = name;
//         this.autoCommand = autoCommand;
//         this.potentialContinuations = potentialContinuations;
//     }

//     public static void clearContinuations(){
//         autoList2.clear();
//     }

//     public static void fillAutosList(AutoCommand auto){
//         for(String name : auto.potentialContinuations){
//             autoList2.add(getAuto(name));
//         }
//     }

//     public ArrayList<AutoCommand> getPotentialContinuations() {
//         return autoList2;
//     }

//     public static AutoCommand Test1(){
//         return new AutoCommand("Test1", Autos.Test1(), new String[]{"Test1", "Test2"});
//     }

//     public static AutoCommand Test2(){
//         return new AutoCommand("Test2", Autos.Test2(), new String[]{"Test1", "Test2"});
//     }

//     public static AutoCommand getAuto(String name){
//         return pathKeys.get(name);
//     }
// }
