// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtensionSubsystem extends SubsystemBase {
  private CANSparkFlex extensionMotor = new CANSparkFlex(13, MotorType.kBrushless);
  private RelativeEncoder extension_encoder;
  private SparkPIDController extension_pidController;
  private CANSparkFlex extensionMotorFollower = new CANSparkFlex(14, MotorType.kBrushless);


  // PID coefficients............................................
   double kP = 0.2; 
   double kI = 0;
   double kD = 0; 
   double kF = 0; 
   double kIz = 0; 
   double kFF = 0; 
   double kMaxOutput = 0.5; //set speed limits if needed
   double kMinOutput = -0.5;  //set speed limits if needed

//PID Setpoint....................................................
   double floor = 19.7; //tune
   double stowed = 15.7; //tune
   double ampScore = 7; //tune


  public ExtensionSubsystem() {
    extensionMotor.restoreFactoryDefaults();
    extensionMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    extensionMotor.setInverted(false); 
    extensionMotorFollower.restoreFactoryDefaults();
    extensionMotorFollower.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    extensionMotorFollower.setInverted(true);
    extensionMotorFollower.follow(extensionMotor);

  
    extensionMotor.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kForward, false);
    extensionMotor.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, false);
  
    extensionMotor.setSoftLimit(CANSparkFlex.SoftLimitDirection.kForward, 15); //what is 15?
    extensionMotor.setSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, 0);
  
    extension_encoder = extensionMotor.getEncoder();
         
    extension_pidController = extensionMotor.getPIDController();
    
    // set PID coefficients
    extension_pidController.setP(kP); 
    extension_pidController.setI(kI);
    extension_pidController.setD(kD);
    extension_pidController.setIZone(kIz);
    extension_pidController.setFF(kFF);
    extension_pidController.setOutputRange(kMinOutput, kMaxOutput);


   SmartDashboard.putNumber("ProcessVariable", extension_encoder.getPosition());
   
  extension_pidController.setReference(floor, CANSparkFlex.ControlType.kPosition);
  extension_pidController.setReference(stowed, CANSparkFlex.ControlType.kPosition);
  extension_pidController.setReference(ampScore, CANSparkFlex.ControlType.kPosition);


  }

  public void floor () {
  extension_pidController.setReference(floor, CANSparkFlex.ControlType.kPosition);
  }

public void stowed () {
  extension_pidController.setReference(stowed, CANSparkFlex.ControlType.kPosition);
  }

public void ampScore () {
   extension_pidController.setReference(ampScore, CANSparkFlex.ControlType.kPosition);
  }

  public void extensionMotorSpeed(double speed) {
    extensionMotor.set(speed);
  }

  public double getExtensionPosition() {
    return extension_encoder.getPosition();
  }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Extension Position", getExtensionPosition());

}
}



