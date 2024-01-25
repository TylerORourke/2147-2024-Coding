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


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
    private CANSparkFlex ArmMotor = new CANSparkFlex(15, MotorType.kBrushless);
    private RelativeEncoder Arm_encoder;
    private SparkPIDController Arm_pidController;
    private CANSparkFlex ArmMotorFollower = new CANSparkFlex(16, MotorType.kBrushless);
  
  
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
     double floorArm = 19.7; //tune
     double stowedArm = 15.7; //tune
     double ampScoreArm = 7; //tune
  
  
    public ArmSubsystem() {
      ArmMotor.restoreFactoryDefaults();
      ArmMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
      ArmMotor.setInverted(false); 
      ArmMotorFollower.restoreFactoryDefaults();
      ArmMotorFollower.setIdleMode(CANSparkFlex.IdleMode.kBrake);
      ArmMotorFollower.setInverted(true);
      ArmMotorFollower.follow(ArmMotor);
  
    
      ArmMotor.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kForward, false);
      ArmMotor.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, false);
    
      ArmMotor.setSoftLimit(CANSparkFlex.SoftLimitDirection.kForward, 15); //what is 15?
      ArmMotor.setSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, 0);
    
      Arm_encoder = ArmMotor.getEncoder();
           
      Arm_pidController = ArmMotor.getPIDController();
      
      // set PID coefficients
      Arm_pidController.setP(kP); 
      Arm_pidController.setI(kI);
      Arm_pidController.setD(kD);
      Arm_pidController.setIZone(kIz);
      Arm_pidController.setFF(kFF);
      Arm_pidController.setOutputRange(kMinOutput, kMaxOutput);
  
  
     SmartDashboard.putNumber("ProcessVariable", Arm_encoder.getPosition());
     
    Arm_pidController.setReference(floorArm, CANSparkFlex.ControlType.kPosition);
    Arm_pidController.setReference(stowedArm, CANSparkFlex.ControlType.kPosition);
    Arm_pidController.setReference(ampScoreArm, CANSparkFlex.ControlType.kPosition);
  
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Object ArmMotorSpeed(double d) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'ArmMotorSpeed'");
  }

  public void floorArm() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'floorArm'");
  }

  public void stowedArm() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stowedArm'");
  }

public void ampScoreArm() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'ampScoreArm'");
}
}
