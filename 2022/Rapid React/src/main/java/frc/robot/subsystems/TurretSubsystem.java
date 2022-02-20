// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.TurretCenter;
import frc.robot.commands.TurretNeutral;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//ha

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  static CANSparkMax m_motor = new CANSparkMax(ShooterConstants.NEO_TURRET_ID, MotorType.kBrushless); //instantiates neo
  public static RelativeEncoder m_encoder = m_motor.getEncoder();
  static double h1 = ShooterConstants.limelight_height; //corresponds to height limelight is off the ground
  static double h2 = ShooterConstants.hub_height; //corresponds to the case study graph in limelight document
  static double a1 = ShooterConstants.limelight_angle; //degrees, corresponds to the case study graph in limelight document; angle between the camera and the ground
  public static double distanceFromHub ; //corresponds to the case study graph in limelight document; horizontal distance between the camera and the target
  //setting weather its a brushed or non brushed motor
  //defines the motor
  public TurretSubsystem() {

}
  public void autoTurn () {
    double tx = getX();//offset on x axis
    double tv = getTv();//is target in view?
    boolean tapeFound;

  
    if (tv != 1) {tapeFound = false;}
    else {tapeFound = true;}
    
    SmartDashboard.putBoolean("limelight vision", tapeFound); //prints if target has been found to dash
    if (tapeFound == true){
  
      //Robot.driveSubsystem.teleop(0, 0);
      double steering_adjust = 0.0f;
      if (tx > 0 ) //if to the right of center, turns left; to test deadzone 
      {
          
        if (m_encoder.getPosition() < -ShooterConstants.max_turret_rotation)
          {
            new TurretCenter();
          }
        else{
          steering_adjust = ShooterConstants.Turret_Kp * tx - ShooterConstants.min_command; //Kp is a number that sets the speed, and the min command is the minimum needed for it to react
          //steering_adjust = -.05; //to test speed and inversion
          turnTurret(steering_adjust); //turns the turret
          }
      }
    
      else if (tx < 0 ) //if to the left of center, turns right; to test deadzone
      {
        if (m_encoder.getPosition() > ShooterConstants.max_turret_rotation)
        {
          new TurretCenter();
        }
        else
          {steering_adjust = ShooterConstants.Turret_Kp * tx + ShooterConstants.min_command;
          //steering_adjust = .05; //to test speed and inversion
          turnTurret(steering_adjust);//turns the turret
          }
      }
  
      else{turnTurret(0);}
  
  
      
    }
    else{turnTurret(0);}
  }



  public void turnTurret (double turnSpeed) {
    m_motor.set(turnSpeed); //makes motor of turret speed given
    printTurretEncoder(); //pushes number of motor rotations to shuffleboard
    getDistance(); // gets the distance from hub

  }

  public void printTurretEncoder ()
  {
    SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
  }

  public static double getDistance () //gets the distance from the hub
  {
    double ty = getY();
    double tx = getX();
    double tv = getTv();
    double angleRad = Math.PI*(ty+a1)/180; // converting ty+a1 from degrees to radians
    distanceFromHub = (h2-h1) / Math.tan(angleRad); // caculate the distance with M A T H and T R I G O N O M E T R Y 
    SmartDashboard.putNumber("DistanceFromHub", distanceFromHub); 
    return distanceFromHub; 

  }

  public static double getY () //gets Y offset of limelight
  {
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);//degrees, offset from target on y axis
    SmartDashboard.putNumber("LimelightY", ty);
    return ty;
  }

  public static double getX () //gets X offset of limelight
  {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);//degrees, offset from target on y axis
    SmartDashboard.putNumber("LimelightX", tx);
    return tx;
  }
 
  public static double getTv () //gets tv of limelight
  {
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);//is target in view?
    SmartDashboard.putNumber("Target Found", tv);
    return tv;
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
