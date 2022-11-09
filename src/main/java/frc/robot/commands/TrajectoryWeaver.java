// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands;
import java.util.HashMap;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

// Runs a given pp-trajectory as a command 
public class TrajectoryWeaver extends SequentialCommandGroup{
    
    // Constructor that obtains required values
    public TrajectoryWeaver(SwerveSubsystem swerveSubsystem, PIDController xController,
    PIDController yController,  PIDController ppthetaController,
    PathPlannerTrajectory pptrajectory, Boolean isFirstPath){
      
        // Create map of events for testing
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        // Tell theta PID controller that its a circle
        ppthetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Check if first path
        if(isFirstPath){
           // Reset robot odometry before movement 
          addCommands(new ResetOdometry(swerveSubsystem, pptrajectory.getInitialPose()));
        }

        addCommands(
            // Commands to run sequentially
            new SequentialCommandGroup(
              // Move robot with path planner swerve command
              new PPSwerveControllerCommand(pptrajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, ppthetaController, swerveSubsystem::setModuleStates, swerveSubsystem),
              // Tell driver station that command is running
              new ReportWarning("Trajectory weaver: " + pptrajectory.toString()),
              // Stop all module movement
              new InstantCommand(() -> swerveSubsystem.stopModules())
            )
        ); 
        
        
        

    }
}