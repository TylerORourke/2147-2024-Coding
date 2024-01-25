// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private CANSparkFlex ShooterMotor = new CANSparkFlex(11, MotorType.kBrushless);
  private CANSparkFlex ShooterMotorFollower = new CANSparkFlex(12, MotorType.kBrushless);
  private RelativeEncoder ShooterMotor_encoder;
  private SparkPIDController ShooterMotor_pidController;
    // PID coefficients............................................
    double kP = 0.2; 
    double kI = 0;
    double kD = 0; 
    double kF = 0; 
    double kIz = 0; 
    double kFF = 0; 
    double kMaxOutput = 0.5; //set speed limits if needed
    double kMinOutput = -0.5;  //set speed limits if needed

    double speaker =0.0;
    double amp =0.0;

  public ShooterSubsystem() {
        ShooterMotor.restoreFactoryDefaults();
        ShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
        ShooterMotor.setInverted(false);  
        ShooterMotorFollower.restoreFactoryDefaults();
        ShooterMotorFollower.setIdleMode(CANSparkFlex.IdleMode.kCoast);
        ShooterMotorFollower.setInverted(true);
        ShooterMotorFollower.follow(ShooterMotor);
        ShooterMotor.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kForward, false);
        ShooterMotor.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, false);
      
        ShooterMotor.setSoftLimit(CANSparkFlex.SoftLimitDirection.kForward, 15); //what is 15?
        ShooterMotor.setSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, 0);
        ShooterMotor_encoder = ShooterMotor.getEncoder();
         
        ShooterMotor_pidController = ShooterMotor.getPIDController();
        ShooterMotor_pidController.setP(kP); 
        ShooterMotor_pidController.setI(kI);
        ShooterMotor_pidController.setD(kD);
        ShooterMotor_pidController.setIZone(kIz);
        ShooterMotor_pidController.setFF(kFF);
        ShooterMotor_pidController.setOutputRange(kMinOutput, kMaxOutput);
        SmartDashboard.putNumber("ProcessVariable", ShooterMotor_encoder.getVelocity());
        ShooterMotor_pidController.setReference(speaker, CANSparkFlex.ControlType.kVelocity);
        ShooterMotor_pidController.setReference(amp, CANSparkFlex.ControlType.kVelocity);

  }
  public void SpeakerShot () {
    ShooterMotor_pidController.setReference(speaker, CANSparkFlex.ControlType.kPosition);
    }
  public void ShooterMotorSpeed(double speed) {
    ShooterMotor.set(speed);
  }
  public void AmpShot () {
    ShooterMotor_pidController.setReference(amp, CANSparkFlex.ControlType.kPosition);
    }
}
