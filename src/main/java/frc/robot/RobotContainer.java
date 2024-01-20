// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunShooterLinkage;
import frc.robot.hardware.IntakeIOSparkMax;
import frc.robot.hardware.ShooterIOSparkMax;
import frc.robot.hardware.ShooterLinkageIOSparkMax;
import frc.robot.io.IntakeIO;
import frc.robot.io.ShooterIO;
import frc.robot.io.ShooterLinkageIO;
import frc.robot.sim.IntakeIOSim;
import frc.robot.sim.ShooterIOSim;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLinkage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER);
  private final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER);

  // declared as final in example code, but gives error in our code
  private Intake intake;
  // private Shooter shooter;
  // private ShooterLinkage shooterLinkage;

  private RunIntake runIntake;
  // private final RunShooter runShooter = new RunShooter();
  // private final RunShooterLinkage runShooterLinkage = new RunShooterLinkage();

  
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // shooter = new Shooter(new ShooterIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        // shooterLinkage = new ShooterLinkage(new ShooterLinkageIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // CHANGE SHOOTER AND SHOOTER LINKAGE TO SIM LATER
        // shooter = new Shooter(new ShooterIOSparkMax());
        intake = new Intake(new IntakeIOSim());
        // shooterLinkage = new ShooterLinkage(new ShooterLinkageIOSparkMax());
        break;

      default:
        // Replayed robot, disable IO implementations
        // shooter = new Shooter(new ShooterIO() {});
        // intake = new Intake(new IntakeIO() {});
        // shooterLinkage = new ShooterLinkage(new ShooterLinkageIO() {});
        break;
    }
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */  


  private void configureBindings() {
  
    runIntake = new RunIntake(intake);

    // operatorController.a().whileTrue(runShooter);
    operatorController.b().whileTrue(runIntake);
    // operatorController.x().whileTrue(new InstantCommand(() -> shooterLinkage.zero(), shooterLinkage));
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }


  private void configureDefaultCommands() {
    // shooter.setDefaultCommand(runShooter);
    // shooterLinkage.setDefaultCommand(runShooterLinkage);
    intake.setDefaultCommand(runIntake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(null);
  // }
}
