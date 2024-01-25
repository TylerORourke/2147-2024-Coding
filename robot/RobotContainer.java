// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ExtensionSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class RobotContainer {

  /* Controllers */
  final CommandXboxController driver = new CommandXboxController (0);
  final CommandXboxController operator = new CommandXboxController (1);

  /* Subsystems */
  final ArmSubsystem armSubsystem = new ArmSubsystem();
  final ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem();
  final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(); 
  final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  
  public RobotContainer() {
       configureButtonBindings(); 
  }

  private void configureButtonBindings() {
    
    //intake forward
    driver.y().whileTrue(new StartEndCommand(() -> intakeSubsystem.IntakeMotorSpeed(.5), 
      () -> intakeSubsystem.IntakeMotorSpeed(0)
      ));
    
    //intake reverse
    driver.a().whileTrue(new StartEndCommand(() -> intakeSubsystem.IntakeMotorSpeed(-.5), 
      () -> intakeSubsystem.IntakeMotorSpeed(0)
      ));

      //SHOOOOOOTTTTT!!!
      driver.x().whileTrue(new StartEndCommand(()-> shooterSubsystem.ShooterMotorSpeed(1),
      ()->shooterSubsystem.ShooterMotorSpeed(0)));

      //arm
      driver.start().whileTrue(new InstantCommand(()->{armSubsystem.floorArm();}));
      driver.back().whileTrue(new InstantCommand(()->{armSubsystem.stowedArm();}));
      driver.b().whileTrue(new InstantCommand(()->{armSubsystem.ampScoreArm();}));
     

    /*Positions Extension*/
    driver.povUp().onTrue(new InstantCommand(() -> {extensionSubsystem.ampScore();}));
    driver.povRight().onTrue(new InstantCommand(() -> {extensionSubsystem.stowed();}));
    driver.povDown().onTrue(new InstantCommand(() -> {extensionSubsystem.floor();}));
   
    //manual extension up
    driver.leftBumper().whileTrue(new StartEndCommand(() -> extensionSubsystem.extensionMotorSpeed(.5), 
      () -> extensionSubsystem.extensionMotorSpeed(0)
      ));
    
    //manual extension down
    driver.rightBumper().whileTrue(new StartEndCommand(() -> extensionSubsystem.extensionMotorSpeed(-.5), 
      () -> extensionSubsystem.extensionMotorSpeed(0)
      ));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
