// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.RunExtendIntake;
import frc.robot.commands.PowerIntakeReversed;
import frc.robot.commands.PowerIntake;
import frc.robot.commands.PowerLinkage;
import frc.robot.commands.SetIntake;
import frc.robot.commands.SetLinkageTalon;
import frc.robot.commands.ShootInSpeaker;
import frc.robot.commands.PowerFlywheel;
import frc.robot.commands.RobotOrientedDrive;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.generated.PracticebotConstants;
import frc.robot.generated.WoodbotConstants;
import frc.robot.hardware.FlywheelIOSparkFlex;
import frc.robot.hardware.IntakeIOSparkMax;
import frc.robot.hardware.LinkageIOTalonFX;
import frc.robot.io.IntakeIO;
import frc.robot.sim.ShooterIOSim;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Linkage;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  // declared as final in example code, but gives error in our code
  private SendableChooser<Command> autoChooser;

  private final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER);
  private final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER);

  final double MaxSpeed = 13.7; // used to be 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI * 3; // Half a rotation per second max angular velocity
  // subsystems
  private final CommandSwerveDrivetrain drivetrain = WoodbotConstants.DriveTrain; // My drivetrain
  private final Flywheel flywheel = new Flywheel(new FlywheelIOSparkFlex());
  private final Linkage linkage = new Linkage(new LinkageIOTalonFX());
  private final Intake intake = new Intake(new IntakeIOSparkMax());
  // private final Climber climber = new Climber(new ClimberIOSparkMax());

  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  private final Command shootRoutine = new ShootInSpeaker(linkage, flywheel, drivetrain, intake, 0.0, 5000.0, 90.0);

  // tele commands
  private RunExtendIntake runExtendIntake = new RunExtendIntake(intake);
  private PowerIntakeReversed powerIntakeReversed = new PowerIntakeReversed(intake);
  private PowerIntake powerIntake = new PowerIntake(intake);
  private PowerFlywheel powerFlywheel = new PowerFlywheel(flywheel);
  private PowerLinkage powerLinkage = new PowerLinkage(linkage);
  private FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive(drivetrain);
  private RobotOrientedDrive robotOrientedDrive = new RobotOrientedDrive(drivetrain);

  private SetLinkageTalon setLinkageTalon = new SetLinkageTalon(linkage);

  final Rotation2d setAngle = Rotation2d.fromDegrees(0);

  /* Setting up bindings for necessary control of the swerve drive platform */
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(Constants.MAX_SPEED_MPS);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // switch (Constants.currentMode) {
    // case REAL:
    // // Real robot, instantiate hardware IO implementations
    // flywheel = new Shooter(new ShooterIOSparkMax());
    // intake = new Intake(new IntakeIOSparkMax());
    // linkage = new Linkage(new LinkageIOSparkMax());
    // // commandSwerveDrivetrain = new CommandSwerveDrivetrain(new
    // CommandSwerveDrivetrainIOSparkMax());
    // break;

    // case SIM:
    // // Sim robot, instantiate physics sim IO implementations
    // // CHANGE SHOOTER AND SHOOTER LINKAGE TO SIM LATER
    // // shooter = new Shooter(new ShooterIOSparkMax());
    // // intake = new Intake(new IntakeIOSparkMax());
    // // linkage = new Linkage(new LinkageIOSparkMax());
    // // commandSwerveDrivetrain = new CommandSwerveDrivetrain(new
    // CommandSwerveDrivetrainIOSparkMax());
    // break;

    // default:
    // // Replayed robot, disable IO implementations
    // // shooter = new Shooter(new ShooterIO() {});
    // // intake = new Intake(new IntakeIO() {});
    // // shooterLinkage = new ShooterLinkage(new ShooterLinkageIO() {});
    // break;
    // }
    // Configure the trigger bindings
    NamedCommands.registerCommand("Intake", runExtendIntake);
    NamedCommands.registerCommand("Wait1", new WaitCommand(1));
    NamedCommands.registerCommand("Wait", new WaitCommand(2));
    NamedCommands.registerCommand("Shoot", shootRoutine);
    NamedCommands.registerCommand("Rotate", drivetrain.turntoCMD(false, 45.0, 0, 0));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    // linkage.setDefaultCommand(powerLinkage);
    //flywheel.setDefaultCommand(powerFlywheel);
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

    operatorController.a().whileTrue(setLinkageTalon);

    // operatorController.leftBumper().whileTrue(powerIntakeReversed);
    // operatorController.rightBumper().whileTrue(powerIntake);

    // operatorController.y().whileTrue(powerFlywheel);
    // // OPERATOR CONTROLLER BINDINGS
    // operatorController.a().toggleOnTrue(runExtendIntake);
    // operatorController.b().whileTrue(powerIntake);
    // operatorController.y().toggleOnTrue(new InstantCommand(() ->
    // flywheel.setBothRPM(5000), flywheel));

    // // DRIVER CONTROLLER BINDINGS
    // driverController.x().whileTrue(new InstantCommand(() -> drivetrain.zero(),
    // drivetrain));

    // drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void onDisable() {
    flywheel.stop();
    drivetrain.robotCentricDrive(0, 0, 0);

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
