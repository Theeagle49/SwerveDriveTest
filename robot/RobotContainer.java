// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.auto.commands.TrajectoryRunner;
import frc.robot.auto.manuals.Forward2M;
import frc.robot.auto.routines.TestRoutine;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.SwerveRotator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.IOConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Ignore unused variable warnings
@SuppressWarnings("unused")

public class RobotContainer {

  // GIT TEST LAPTOP
  //------------------------------------O-B-J-E-C-T-S-----------------------------------//

  // Create joysticks
  private final Joystick leftJoystick = new Joystick(IOConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(IOConstants.kRightJoystick);

  // Create swerve subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(rightJoystick);

  // Create Xbox controller
  private final XboxController xboxController = new XboxController(IOConstants.kXboxController);

  // Create PID controllers for trajectory tracking
  private final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  private final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  // Create a non profiled PID controller for path planner
  private final PIDController ppThetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);

  //------------------------------------C-O-N-S-T-R-U-C-T-O-R----------------------------//

  public RobotContainer(){

    // Set swerve subsystem default command to swerve joystick with respective joystick inputs
    // Axies: 0 = left/right : 1 = forward/backwards : 2 = dial
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> rightJoystick.getRawAxis(0 /* Place axis value here! X-AXIS */),
    () -> rightJoystick.getRawAxis(1 /* Place axis value here! Y-AXIS */),
    //() -> leftJoystick.getRawAxis(0 /* Place axis value here! R-AXIS */),
    () -> rightJoystick.getRawAxis(2),
    () -> !leftJoystick.getRawButton(Constants.IOConstants.kFieldOrientedButton /* Field oriented? */)));

    // Run button binding method
    configureButtonBindings();

    
  }

  //------------------------------------B-U-T-T-O-N-S------------------------------------//

  // Create buttons bindings
  private void configureButtonBindings(){

    // Assign button to manually zero heading
    new JoystickButton(rightJoystick,Constants.IOConstants.kZeroHeadingButton).whenPressed(() -> swerveSubsystem.zeroHeading());

    // Rotate robot 90* using swerve rotator
    //new JoystickButton(leftJoystick, Constants.IOConstants.kRotatorButton).whenPressed(new SwerveRotator(swerveSubsystem, () -> 0.1, swerveSubsystem.getHeading()));

  }

    //------------------------------------R-E-F-E-R-R-E-R-S------------------------------------//

    public void containerResetAllEncoders() {
      DriverStation.reportWarning("Running containerResetAllEncoders() in RobotContainer", true);
      swerveSubsystem.resetAllEncoders();
    }

  //------------------------------------A-U-T-O-N-O-M-O-U-S------------------------------------//
  
  // Create a command using TrajectoryRunner and pass in the trajectory to run
  private Command forward2M = new TrajectoryRunner(swerveSubsystem, xController, yController, thetaController, Forward2M.getTrajectory(), Forward2M.getTrajectoryConfig());
    
  // Load in test routine command for auto selector
  private Command testRoutine = new TestRoutine(swerveSubsystem, xController, yController, ppThetaController);

  // Returns command to run during auto
  public Command getAutonomousCommand(){

    String autoSelector = "forward2M";
    Command autoCommand = null;

  //------------------------------------S-E-L-E-C-T-O-R------------------------------------//

    // Selector if-statement
    if(autoSelector == "forward2M"){
      autoCommand = forward2M;
    }
    else if(autoSelector == "testRoutine"){
      autoCommand = testRoutine;
    }

    return autoCommand;
  }

  public SwerveSubsystem getSwerveSubsystem(){
    return swerveSubsystem;
  }
}

