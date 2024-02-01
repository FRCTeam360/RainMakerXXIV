// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.RunExtendIntake;
import frc.robot.commands.PowerIntakeReversed;
import frc.robot.commands.PowerFlywheel;
import frc.robot.commands.PowerIntake;
import frc.robot.commands.PowerLinkage;
import frc.robot.commands.RobotOrientedDrive;
//import frc.robot.commands.SetFlywheel;
import frc.robot.commands.SetLinkage;
import frc.robot.commands.ShootInSpeaker;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Linkage;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER);
  private final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER);

  // subsystems
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Intake intake = Intake.getInstance();
  private final Flywheel flywheel = Flywheel.getInstance();
  private final Linkage linkage = Linkage.getInstance();

  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();


  // auto commands
  //private final SetFlywheel setFlywheel = new SetFlywheel();

  // tele commands
  private final RunExtendIntake runExtendIntake = new RunExtendIntake();
  private final PowerIntakeReversed powerIntakeReversed = new PowerIntakeReversed();
  private final PowerIntake powerIntake = new PowerIntake();
  private final PowerFlywheel powerFlywheel = new PowerFlywheel();
  private final PowerLinkage powerLinkage = new PowerLinkage();
  private final SetLinkage setLinkage = new SetLinkage();
  private final FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive();
  private final RobotOrientedDrive robotOrientedDrive = new RobotOrientedDrive();
  


  // public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  //     .withDeadband(Constants.MAX_SPEED * 0.1).withRotationalDeadband(Constants.MAX_ANGULAR_RATE * 0.1)
  //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // // driving in open loop
  
  final Rotation2d setAngle = Rotation2d.fromDegrees(0);

  /* Setting up bindings for necessary control of the swerve drive platform */
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(Constants.MAX_SPEED_MPS);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    NamedCommands.registerCommand("RunExtendIntake", runExtendIntake);
    NamedCommands.registerCommand("Wait1", new WaitCommand(1));
    NamedCommands.registerCommand("Shoot", powerFlywheel);
    NamedCommands.registerCommand("Rotate", drivetrain.turntoCMD(45.0, 0, 0));
    configureBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    //flywheel.setDefaultCommand(setFlywheel);
    //intake.setDefaultCommand(runIntake);
    //linkage.setDefaultCommand(powerLinkage);
    drivetrain.setDefaultCommand(fieldOrientedDrive);
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.a().whileTrue(drivetrain.turntoCMD(180.0, 0.0, 0.0));
    driverController.x().whileTrue(new InstantCommand(() -> drivetrain.zero(), drivetrain));

    // OPERATOR CONTROLLER BINDINGS
    operatorController.leftTrigger(.005).whileTrue(powerIntakeReversed);
    operatorController.rightTrigger(.005).whileTrue(powerIntake);
    operatorController.a().toggleOnTrue(runExtendIntake);
    operatorController.b().whileTrue(powerIntake);
    operatorController.y().toggleOnTrue(new InstantCommand(() -> flywheel.setBothRPM(5000), flywheel));

    // DRIVER CONTROLLER BINDINGS
    // driverController.x().whileTrue(new InstantCommand(() -> drivetrain.xOut(), drivetrain));
    // driverController.a().whileTrue(new InstantCommand(() -> drivetrain.zero(), drivetrain));
    
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void onDisable() {
    flywheel.stop();
        drivetrain.robotOrientedDrive(0, 0, 0);

  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // An example command will be run in autonomous
  }
}
