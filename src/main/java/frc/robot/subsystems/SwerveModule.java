// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.ModuleConstants;

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

  private String moduleName;

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

    // Change drive turning conversion factors
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    // Create PID controller
    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);

    // Tell PID controller that it is a *wheel*
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    //absoluteEncoder.setDutyCycleRange(1/4096, 4095/4096);
// stephen join robo

    // Call resetEncoders
    resetEncoders();

  }

  public void update(){

    SmartDashboard.putNumber(moduleName + " ABE Absolute Position", absoluteEncoder.getAbsolutePosition());
    SmartDashboard.putNumber(moduleName + " ABE Radians Raw", absoluteEncoder.getAbsolutePosition() * 2.0 * Math.PI);

    SmartDashboard.putNumber(moduleName + " ABE Radians", getAbsoluteEncoderRad());

    SmartDashboard.putNumber(moduleName + " Drive Position", getDrivePosition());
    SmartDashboard.putNumber(moduleName + " Turning Position", getTurningPosition());

    SmartDashboard.putNumber(moduleName + " Drive Velocity", getDriveVelocity());
    SmartDashboard.putNumber(moduleName + " Turning Velocity", getTurningVelocity());

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
    double angle = 0;

    // Get encoder absolute position goes from 1 to 0
    angle = absoluteEncoder.getAbsolutePosition();

    // Convert into radians
    angle *= 2.0 * Math.PI;

    // Apply magnetic offsets in radians
    angle -= absoluteEncoderOffsetRad;

    // Mulitply by -1 if angle is less than 0
    if(angle < 0){
      angle = 2.0 * Math.PI + angle ;
    }

    angle = Math.abs(angle);

    // Make negative if needed
    return angle * ( absoluteEncoderReversed ? -1.0 : 1.0);
    
  }

  // Set turning encoder to match absolute encoder value with gear offsets applied
  public void resetEncoders(){
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(absoluteEncoder.getAbsolutePosition());
  }

  // Get swerve module current state, aka velocity and wheel rotation
  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state){
    
    // Check if new command has high driving power 
    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }

    // Optimize swerve module state to do fastest rotation movement, aka never rotate more than 90*
    state = SwerveModuleState.optimize(state, getState().angle);

    // Scale velocity down using robot max speed
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    // Use PID to calculate angle setpoint
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

  }

  // Stop all motors on module 
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

}
