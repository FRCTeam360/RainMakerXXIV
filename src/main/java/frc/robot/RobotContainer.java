// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotType;
import frc.robot.commands.Autos;
import frc.robot.commands.RunExtendIntake;
import frc.robot.commands.PowerIntakeReversed;
import frc.robot.commands.PowerIntake;
import frc.robot.commands.PowerLinkage;
import frc.robot.commands.SetIntake;
import frc.robot.commands.ShootInSpeaker;
import frc.robot.commands.PowerFlywheel;
import frc.robot.commands.RobotOrientedDrive;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.hardware.FlywheelIOSparkFlex;
import frc.robot.hardware.IntakeIOSparkMax;
import frc.robot.hardware.LinkageIOSparkMax;
import frc.robot.io.FlywheelIO;
import frc.robot.io.IntakeIO;
import frc.robot.sim.ShooterIOSim;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Linkage;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
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
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private SendableChooser<Command> autoChooser;
  private final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER);
  private final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER);

  final double MaxSpeed = 13.7; // used to be 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI * 3; // Half a rotation per second max angular velocity
  // subsystems
  private CommandSwerveDrivetrain drivetrain; // My drivetrain
  private Flywheel flywheel;
  private Linkage linkage;
  private Intake intake;
  // private final Climber climber = new Climber(new ClimberIOSparkMax());


  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  // Create new ShootInSpeaker command
  private final Command shootRoutine = new ShootInSpeaker(linkage, flywheel, drivetrain, intake, 0.0, 5000.0, 90.0);
   // auto commands
  //private final SetFlywheel setFlywheel = new SetFlywheel();

  // tele commands
  private RunExtendIntake runExtendIntake;
  private PowerIntakeReversed powerIntakeReversed;
  private PowerIntake powerIntake;
  private PowerFlywheel powerFlywheel;
  // private PowerLinkage powerLinkage = new PowerLinkage(linkage);
  private FieldOrientedDrive fieldOrientedDrive;
  private RobotOrientedDrive robotOrientedDrive;
  


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
    switch (Constants.getRobotType()) {
      case WOODBOT:
        // Real robot, instantiate hardware IO implementations
        flywheel = new Flywheel(new FlywheelIOSparkFlex());
        intake = new Intake(new IntakeIOSparkMax());
        linkage = new Linkage(new LinkageIOSparkMax());
        drivetrain = TunerConstants.DriveTrain; // My drivetrain
        // commandSwerveDrivetrain = new CommandSwerveDrivetrain(new CommandSwerveDrivetrainIOSparkMax());
        break;
      case PRACTICE:

        break;
      case COMPETITION:

        break; 
      case TEST:

        break; 
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // CHANGE SHOOTER AND SHOOTER LINKAGE TO SIM LATER
        // shooter = new Shooter(new ShooterIOSparkMax());
        // intake = new Intake(new IntakeIOSparkMax());
        // linkage = new Linkage(new LinkageIOSparkMax());
        // commandSwerveDrivetrain = new CommandSwerveDrivetrain(new CommandSwerveDrivetrainIOSparkMax());
        break;
      case REPLAY:

        break;

      default:
        // Replayed robot, disable IO implementations
        // shooter = new Shooter(new ShooterIO() {});
        // intake = new Intake(new IntakeIO() {});
        // shooterLinkage = new ShooterLinkage(new ShooterLinkageIO() {});
        break;
    }
    initializeCommands();
    NamedCommands.registerCommand("Intake", runExtendIntake);
    NamedCommands.registerCommand("Wait1", new WaitCommand(1));
    NamedCommands.registerCommand("Shoot", shootRoutine);
    NamedCommands.registerCommand("Rotate", drivetrain.turntoCMD(false, 45.0, 0, 0));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
    configureDefaultCommands();
  }

  private final void initializeCommands() {
    runExtendIntake = new RunExtendIntake(intake);
    powerIntakeReversed = new PowerIntakeReversed(intake);
    powerIntake = new PowerIntake(intake);
    powerFlywheel = new PowerFlywheel(flywheel);
    // private PowerLinkage powerLinkage = new PowerLinkage(linkage);
    fieldOrientedDrive = new FieldOrientedDrive();
    robotOrientedDrive = new RobotOrientedDrive();
    drivetrain = TunerConstants.DriveTrain; // My drivetrain
  }

  private void configureDefaultCommands() {
    // flywheel.setDefaultCommand(powerFlywheel);
    // intake.setDefaultCommand(powerIntake);
    // linkage.setDefaultCommand(powerLinkage);
    // // climber.setDefaultCommand();
    //  drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(
    //         () -> drive.withVelocityX(MathUtil.applyDeadband(-driverController.getLeftY(), 0.1) * MaxSpeed) //drive forward with negative y
    //             // negative Y (forward)
    //             .withVelocityY(MathUtil.applyDeadband(-driverController.getLeftX(), 0.1) * MaxSpeed) // drive left with negative x
    //             .withRotationalRate(MathUtil.applyDeadband(-driverController.getRightX(), 0.1) * MaxAngularRate) // drive counterclockwise with negative x                                                                                                  
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
    driverController.a().whileTrue(drivetrain.turntoCMD(false, 90.0, 0.0, 0.0));
  
    // powerIntake = new PowerIntake(intake);
    // powerIntakeReversed = new PowerIntakeReversed(intake);
    // // powerLinkage = new PowerLinkage(linkage);
    // powerFlywheel = new PowerFlywheel(flywheel);
    // runExtendIntake = new RunExtendIntake(intake);
    // powerLinkage = new PowerLinkage(linkage);
    // setLinkage = new SetLinkage(linkage);
    // not sure whether adding in the speed and velocity parameters will make a difference, but it won't work without them
    // setpointFlywheel = new SetpointFlywheel(0.0, shooter);
    // setFlywheel = new SetFlywheel(0.0, shooter);
    
    // operatorController.rightTrigger(.005).whileTrue(powerIntake);
    // operatorController.leftTrigger(.005).whileTrue(powerIntakeReversed);
    // operatorController.leftBumper().whileTrue(runIntake);
    // operatorController.rightBumper().whileTrue(runIntakeReversed);
    // operatorController.a().whileTrue(powerFlywheel);
    // operatorController.x().whileTrue(new InstantCommand(() -> linkage.zero(), linkage));
    
    // driverController.x().whileTrue(new InstantCommand(() -> drivetrain.zero(), drivetrain));

    operatorController.a().whileTrue(shootRoutine);
    // operatorController.a().whileTrue(drivetrain.turntoCMD(false, 90.0, 0.0, 0.0));

    

    // operatorController.x().whileTrue(new InstantCommand(() -> flywheel.runBottom(0.8), flywheel));
    // operatorController.a().whileTrue(new InstantCommand(() -> flywheel.setBottomRPM((2000))));

    // operatorController.pov(0).whileTrue(new InstantCommand(() -> flywheel.setBothRPM(5000)));
    // OPERATOR CONTROLLER BINDINGS
    // operatorController.leftTrigger(.005).whileTrue(powerIntakeReversed);
    // operatorController.rightTrigger(.005).whileTrue(powerIntake);
    // operatorController.a().whileTrue(new SetFlywheel());
    // operatorController.b().toggleOnTrue(runExtendIntake);


    // driverController.a().whileTrue(drivetrain.turntoCMD(true, 180.0, 0.0, 0.0));
    // driverController.x().whileTrue(new InstantCommand(() -> drivetrain.zero(), drivetrain));
    
    // operatorController.y().whileTrue(new InstantCommand(() -> flywheel.runBoth(0.8), flywheel));
    // operatorController.a().whileTrue(new InstantCommand(() -> flywheel.setBothRPM((4000)), flywheel));
    // operatorController.b().whileTrue(new InstantCommand(() -> flywheel.setTopRPM((2000))));

    // operatorController.x().whileTrue(new InstantCommand(() -> flywheel.runBottom(0.8), flywheel));
    // operatorController.a().whileTrue(new InstantCommand(() -> flywheel.setBottomRPM((2000))));

    // operatorController.pov(0).whileTrue(new InstantCommand(() -> flywheel.setBothRPM(5000)));
    // // OPERATOR CONTROLLER BINDINGS
    // operatorController.leftTrigger(.005).whileTrue(powerIntakeReversed);
    // operatorController.rightTrigger(.005).whileTrue(powerIntake);
    // // operatorController.a().whileTrue(new SetFlywheel());
    operatorController.b().toggleOnTrue(runExtendIntake);
    // operatorController.y().whileTrue(powerIntake);
    // //operatorController.x().whileTrue(new InstantCommand(() -> linkage.setTo90(),linkage));
    // OPERATOR CONTROLLER BINDINGS
    // operatorController.leftTrigger(.005).whileTrue(powerIntakeReversed);
    // operatorController.rightTrigger(.005).whileTrue(powerIntake);
    // operatorController.a().toggleOnTrue(runExtendIntake);
    // operatorController.b().whileTrue(powerIntake);
    // operatorController.y().toggleOnTrue(new InstantCommand(() -> flywheel.setBothRPM(5000), flywheel));

    // DRIVER CONTROLLER BINDINGS
    // driverController.x().whileTrue(new InstantCommand(() -> drivetrain.xOut(), drivetrain));
    // driverController.a().whileTrue(new InstantCommand(() -> drivetrain.zero(), drivetrain));
    
    drivetrain.registerTelemetry(logger::telemeterize);
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
