// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.sql.Driver;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class SwerveModule extends SubsystemBase {
 
  // Create empty variables for reassignment
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final PIDController turningPidController;

  private final DutyCycleEncoder absoluteEncoder;

  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  private SparkMaxPIDController mTurnController;

  private String moduleName;

  private MechanismLigament2d mTurn;
  private MechanismLigament2d mDirection;

  private MechanismLigament2d mTurn2;
  private MechanismLigament2d mDirection2;

  public static final double kTurningP = 0.01;
  public static final double kTurningI = 0.0;
  public static final double kTurningD = 0.005;

  // Class constructor where we assign default values for variables
   public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoLuteEncoderReversed, String name) {

    // Set offsets for absolute encoder in RADIANS!!!!!
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoLuteEncoderReversed;

    moduleName = name;

    // Create absolute encoder
    absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);

    // Set duty cycle range of encoder
    absoluteEncoder.setDutyCycleRange(1.0/4096.0, 4095.0/4096.0);

    // Create drive and turning motor
    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    // Set reverse state of drive and turning motor
    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    // Set drive and turning motor encoder values
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    // Change drive motor conversion factors
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    // Change conversion factors for neo turning encoder - should be in radians!
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    // Create PID controller
    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);

    // Tell PID controller that it is a *wheel*
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    mTurnController = turningMotor.getPIDController();

    mTurnController.setP(kTurningP);
    mTurnController.setI(kTurningI);
    mTurnController.setD(kTurningD);
    mTurnController.setIZone(0.0);
    mTurnController.setFF(0.0);
    mTurnController.setOutputRange(-1, 1);

    // Set duty cycle for ABE encoder - lasted checked not working correctly!
    //absoluteEncoder.setDutyCycleRange(1/4096, 4095/4096);

   


    // Call resetEncoders
    resetEncoders();

    //Create the mechanism 2d canvas and getting the root
    Mechanism2d mod = new Mechanism2d(6,6);
    MechanismRoot2d root = mod.getRoot("climber", 3, 3);

    //Add mTurn to the root, add direction to turn, put it on smartdashboard
    mTurn = root.append(new MechanismLigament2d("Swerve Turn", 2, 1.75));
    mDirection = mTurn.append(new MechanismLigament2d("Wheel direction", 1, 0, 6, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData(moduleName+" commanded Turn", mod);

  
    //Same thing but to be used for the actual module state
    Mechanism2d mod2 = new Mechanism2d(6,6);
    MechanismRoot2d root2 = mod2.getRoot("climber2", 3, 3);

    mTurn2 = root2.append(new MechanismLigament2d("Swerve Turn", 2, 1.75));
    mDirection2 = mTurn2.append(new MechanismLigament2d("Wheel direction", 1, 0, 6, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData(moduleName+"  real Turn", mod);

  
    

  }

  public void update(){

    SmartDashboard.putNumber(moduleName + "Absolute-Position", absoluteEncoder.getAbsolutePosition());
    SmartDashboard.putNumber(moduleName + "Radians-Raw" , absoluteEncoder.getAbsolutePosition() * 2.0 * Math.PI);
    SmartDashboard.putNumber(moduleName + "Radians", getAbsoluteEncoderRad());

    //SmartDashboard.putNumber(moduleName + " Drive Position", getDrivePosition());
    //SmartDashboard.putNumber(moduleName + " Turning Position", getTurningPosition());

    //SmartDashboard.putNumber(moduleName + " Drive Velocity", getDriveVelocity());
    //SmartDashboard.putNumber(moduleName + " Turning Velocity", getTurningVelocity());

  }

  // Helpful get methods
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
      return turningEncoder.getPosition();
    }

  public double getDriveVelocity() {
      return driveEncoder.getVelocity();
    }

  public double getTurningVelocity() {
      return turningEncoder.getVelocity();
    }

  /* Convert absolute value of the encoder to radians and then subtract the radian offset
  then check if the encoder is reversed.*/
  public double getAbsoluteEncoderRad(){

    //  Make angle variable
    double angle;

    // Get encoder absolute position goes from 1 to 0
    angle = absoluteEncoder.getAbsolutePosition();

    // Convert into radians
    angle *= 2.0 * Math.PI;

    // Apply magnetic offsets in radians
    angle -= absoluteEncoderOffsetRad;

    /*
    if(angle < 0){
      angle = 2.0 * Math.PI + angle ;
    } 
    */

    angle = Math.abs(angle);

    // Make negative if set
    angle *= ( absoluteEncoderReversed ? -1.0 : 1.0);
    
    // Report setting to driver station
    //DriverStation.reportError(moduleName + " called getAbsoluteEncoderRad: " + angle + "  " + absoluteEncoderOffsetRad, true);

    // Return angle in radians for neo turning motor encoder
    return angle;
    
  }

  // Set turning encoder to match absolute encoder value with gear offsets applied
  public void resetEncoders(){
    driveEncoder.setPosition(0);
    REVLibError error = turningEncoder.setPosition(getAbsoluteEncoderRad());
    if(error.value != 0){
      DriverStation.reportError(moduleName + " reset encoders error!: " + error.value, true);
    }
    else if(error.value == 0){
      DriverStation.reportWarning(moduleName + " reset encoders has been ran without errors: " + getAbsoluteEncoderRad(), true);
    }
  }

  // Get swerve module current state, aka velocity and wheel rotation
  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredStateFromAbs(SwerveModuleState state){
    
    // Check if new command has high driving power 
    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }

    // Optimize swerve module state to do fastest rotation movement, aka never rotate more than 90*
    //state = SwerveModuleState.optimize(state, getState().angle);

    // Scale velocity down using robot max speed
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    // Use PID to calculate angle setpoint
    turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));

    mTurn.setAngle(state.angle);
    mDirection.setAngle(state.speedMetersPerSecond>0? 0:180);

    mTurn2.setAngle(absoluteEncoder.getAbsolutePosition());
    mDirection2.setAngle(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond >0 ? 0:180);


  }

  public void setDesiredStateFromInternal(SwerveModuleState state){

    // Check if new command has high driving power 
    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }

    // Optimize swerve module state to do fastest rotation movement, aka never rotate more than 90*
    state = SwerveModuleState.optimize(state, getState().angle);

    // Scale velocity down using robot max speed
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);


    mTurnController.setReference((state.angle.getRadians())/2*Math.PI, CANSparkMax.ControlType.kPosition);
 

    mTurn.setAngle(state.angle);
    mDirection.setAngle(state.speedMetersPerSecond>0? 0:180);

    mTurn2.setAngle(absoluteEncoder.getAbsolutePosition());
    mDirection2.setAngle(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond >0 ? 0:180);
  }

  
  // Stop all motors on module 
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }


  public void simulationPeriodic(){
  

  }

}
