// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.RunExtendIntake;
import frc.robot.commands.PowerIntakeReversed;
import frc.robot.commands.PowerShooter;
import frc.robot.commands.PowerIntake;
import frc.robot.commands.PowerLinkage;
import frc.robot.commands.SetFlywheel;
import frc.robot.commands.SetLinkage;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Linkage;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER);
  private final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER);

  // subsystems
  private final Intake intake = Intake.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Linkage linkage = Linkage.getInstance();

  // auto commands
  private final SetFlywheel setFlywheel = new SetFlywheel(0);

  // tele commands
  private final RunExtendIntake runIntake = new RunExtendIntake();
  private final PowerIntakeReversed manualIntakeReversed = new PowerIntakeReversed();
  private final PowerIntake manualIntake = new PowerIntake();
  private final PowerShooter runShooter = new PowerShooter();
  private final PowerLinkage runShooterLinkage = new PowerLinkage();
  private final SetLinkage setLinkage = new SetLinkage();

  final double MaxSpeed = 13.7; // used to be 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI * 3; // Half a rotation per second max angular velocity

  final Rotation2d setAngle = Rotation2d.fromDegrees(0);

  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(MaxSpeed);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    // shooter.setDefaultCommand(runShooter);
    // intake.setDefaultCommand(runIntake);
    linkage.setDefaultCommand(runShooterLinkage);
     drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () -> drive.withVelocityX(MathUtil.applyDeadband(-driverController.getLeftY(), 0.1) * MaxSpeed) //drive forward with negative y
                // negative Y (forward)
                .withVelocityY(MathUtil.applyDeadband(-driverController.getLeftX(), 0.1) * MaxSpeed) // drive left with negative x
                .withRotationalRate(MathUtil.applyDeadband(-driverController.getRightX(), 0.1) * MaxAngularRate) // drive counterclockwise with negative x                                                                                                  
    ));
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

    operatorController.rightTrigger(.005).whileTrue(manualIntake);
    operatorController.leftTrigger(.005).whileTrue(manualIntakeReversed);
    // operatorController.leftBumper().whileTrue(runIntake);
    // operatorController.rightBumper().whileTrue(runIntakeReversed);
    operatorController.a().whileTrue(runShooter);
    operatorController.x().whileTrue(new InstantCommand(() -> linkage.zero(), linkage));
    
    // DRIVER CONTROLLER BINDINGS
    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.b().whileTrue(drivetrain
        .applyRequest(
            () -> point.withModuleDirection(new Rotation2d(MathUtil.applyDeadband(-driverController.getLeftY(), 0.1),
                MathUtil.applyDeadband(-driverController.getLeftX(), 0.1)))));
    driverController.rightBumper().whileTrue(drivetrain.turntoCMD(setAngle, 0.0, 0.0));

    // if (Utils.isSimulation()) {
    // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
    // Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An example command will be run in autonomous
  // return Autos.exampleAuto(null);
  // }
}
