package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    private static SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle();
    private PhoenixPIDController headingController = new PhoenixPIDController(5.75, 0, 0);

    GenericEntry kPEntry;
    GenericEntry kIEntry;
    GenericEntry kDEntry;

    private double headingKP = 5.75;
    private double headingKI = 0.0;
    private double headingKD = 0.0;

    private void setupShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("angle");
        tab.addNumber("current angle", () -> this.getPigeon2().getAngle());
        tab.addNumber("error", () -> this.getAngleError());
        tab.addNumber("setpoint", () -> this.headingController.getSetpoint());

        // kPEntry = tab.add("kP", 0.0).getEntry();
        // kIEntry = tab.add("kI", 0.0).getEntry();
        // kDEntry = tab.add("kD", 0.0).getEntry();

    }

    private void configurePID() {
        headingController = new PhoenixPIDController(headingKP, headingKI, headingKD);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(5));
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePID();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.75, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        setupShuffleboard();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePID();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.75, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        setupShuffleboard();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command turntoCMD(boolean shouldEnd, Rotation2d desiredAngle, double velocityX, double velocityY) {
        FieldCentricFacingAngle facingAngleCommand = drive.withTargetDirection(desiredAngle).withVelocityX(velocityX)
                .withVelocityY(velocityY);
        facingAngleCommand.HeadingController = headingController;

        if(shouldEnd) {
            return this.applyRequest(() -> facingAngleCommand)
                    .raceWith(new EndWhenFacingAngle(headingController));


        }

        return this.applyRequest(() -> facingAngleCommand);

    }

    public Command turntoCMD(boolean shouldEnd, double desiredAngle, double velocityX, double velocityY) {
        Rotation2d rotation = Rotation2d.fromDegrees(desiredAngle);
        return turntoCMD(shouldEnd, rotation, velocityX, velocityY);

    }

    private double getAngleError() {
        return headingController.getPositionError();
    }

    public boolean isFacingAngle() {
        return headingController.atSetpoint();
    }

    public float getAngle() {
        return (float) this.getPigeon2().getAngle();
    }

    public Rotation2d getRotation2d() {
        return this.getPigeon2().getRotation2d();
    }

    public void zero() {
        this.getPigeon2().reset();
    }

    public void xOut() {
        this.setControl(new SwerveRequest.SwerveDriveBrake());
    }

    public void fieldOrientedDrive(double leftX, double leftY, double rightX) {
        this.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(MathUtil.applyDeadband(leftY, 0.1) * Constants.MAX_SPEED_MPS)
                .withVelocityY(MathUtil.applyDeadband(leftX, 0.1) * Constants.MAX_SPEED_MPS)
                .withRotationalRate(MathUtil.applyDeadband(-rightX, 0.1) * Constants.MAX_ANGULAR_RATE));
    }

    public void robotOrientedDrive(double leftX, double leftY, double rightX) {
        this.setControl(new SwerveRequest.RobotCentric()
                .withVelocityX(MathUtil.applyDeadband(leftY, 0.1) * Constants.MAX_SPEED_MPS)
                .withVelocityY(MathUtil.applyDeadband(leftX, 0.1) * Constants.MAX_SPEED_MPS)
                .withRotationalRate(MathUtil.applyDeadband(-rightX, 0.1) * Constants.MAX_ANGULAR_RATE));
    }

    private Pose2d getPose() {
        // double x = this.getState().Pose.getX();
        // double y = this.getState().Pose.getY();
        // Rotation2d rot = this.getState().Pose.getRotation();
        // System.out.println("CURRENT POSE X: " + x);
        // System.out.println("CURRENT POSE Y: " + y);
        // System.out.println("CURRENT POSE ROTATION: " + rot);
        return this.getState().Pose;
    }

    private void resetPose(Pose2d pose) {
        seedFieldRelative(pose);
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = this.getModule(i).getCurrentState();
        }
        ChassisSpeeds k = this.m_kinematics.toChassisSpeeds(states);
        double x = k.vxMetersPerSecond;
        double y = k.vyMetersPerSecond;
        System.out.println("X VELOCITY: " + x);
        System.out.println("Y VELOCITY: " + y);
        return k;
    }

    private void driveRobotRelative(ChassisSpeeds speed) {
        // print x and y speeds and rotation rate
        // System.out.println("X VELOCITY: " + speed.vxMetersPerSecond);
        // System.out.println("Y VELOCITY: " + speed.vyMetersPerSecond);
        // System.out.println("ROTATION RATE: " + speed.omegaRadiansPerSecond);
        this.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speed));
    }

    private class EndWhenFacingAngle extends Command {
        PhoenixPIDController headingController;

        public EndWhenFacingAngle(PhoenixPIDController headingController) {
            this.headingController = headingController;
        }

        @Override
        public boolean isFinished() {
            return headingController.atSetpoint();
        }
    }

    @Override
    public void periodic() {
        for(int i = 0;i<4;i++) {
           Logger.recordOutput("Voltage", this.getModule(i).getDriveMotor().getMotorVoltage().getValueAsDouble());
           Logger.recordOutput("Current", this.getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble());
           Logger.recordOutput("Position", this.getModule(i).getCANcoder().getPosition().getValueAsDouble());
        }
        Logger.recordOutput("Rotation2d", this.getPigeon2().getRotation2d());
    }

}