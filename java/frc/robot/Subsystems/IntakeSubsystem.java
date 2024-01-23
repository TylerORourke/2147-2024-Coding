// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkFlex intakeMotor = new CANSparkFlex(1, MotorType.kBrushless);

  public IntakeSubsystem() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        intakeMotor.setInverted(false); 
  }

  public void IntakeMotorSpeed(double speed) {
    intakeMotor.set(speed);
  }


}
