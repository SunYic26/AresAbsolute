// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.RobotContainer;

// public class CancelableCommand extends Command {

//     RobotContainer controller;
    
//     public CancelableCommand() {
//         controller = RobotContainer.getInstance();
//     }
  
//     public void initialize() {
  
//     }
  
//     public void execute() {// polls for stick input and gives control back to the driver
//         if(controller.pollInput())  
//             this.end(true);
//         else {}
//     }//idk
  
//     public void end(boolean interrupted) {
//         if (interrupted) {
//             System.out.println("Command interrupted, cancelling commands");
//             CommandScheduler.getInstance().cancelAll();
//         } 
//         //idk if cancel all will have unforseen concquences 
//         // but if it does we can change it to only kill parent
//     }
  
//     public boolean isFinished() {
//       return false;
//     }
// }