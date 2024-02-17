package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

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
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Telemetry;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
    final double MaxSpeed = 13.7;
    boolean shouldUseVisionData =false; 

    CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    private static SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle();
    private PhoenixPIDController headingController;
private Photon vision ;

    GenericEntry kPEntry;
    GenericEntry kIEntry;
    GenericEntry kDEntry;
    // Create a new SysId Routine for characterizing the drive 
    public SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
        null, null, null, // Use default config
        (state) -> Logger.recordOutput("SysIdTestState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
        (voltage) -> this.runCharacterizationVolts(voltage.in(Volts)),
        null, // No log consumer, since data is recorded by AdvantageKit
        this
        )
    );
  
    // See Robot Container: The methods below return Command objects

    public void runCharacterizationVolts (double voltage) {
        double velocityX = voltage / 12.0 * MaxSpeed; 
        this.setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(velocityX)
            .withVelocityY(0.0)
            .withRotationalRate(0.0));
    }

    private double headingKP = 2.5;
    private double headingKI = 0.2;
    private double headingIZone = 0.17;
    private double headingKD = 0.0;

    private void setupShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("angle");
        tab.addNumber("current angle", () -> this.getPigeon2().getAngle());
        tab.addNumber("error", () -> this.getAngleError());
        tab.addNumber("setpoint", () -> Math.toDegrees(this.headingController.getSetpoint()));

        // kPEntry = tab.add("kP", 0.0).getEntry();
        // kIEntry = tab.add("kI", 0.0).getEntry();
        // kDEntry = tab.add("kD", 0.0).getEntry();

    }

    private void configurePID() {
        headingController = new PhoenixPIDController(headingKP, headingKI, headingKD);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(5));
        headingController.setIntegratorRange(-headingIZone, headingIZone);
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

        

        //    synchronized (this.m_odometry) {
             //   for (int i = 0; i < vision.getEstimatedGlobalPose()) {
                    // this.m_odometry.addVisionMeasurement(
                    //         vision.getEstimatedGlobalPose().orElseThrow() ,  /// this is really bad but i think it works, will break robot
                    //         vision.getTimeStamp(), 
                    //         VecBuilder.fill(vision.getEstimationStdDevs(vision.getEstimatedGlobalPose().orElseThrow())  ) ) ;
                    //                 // vision.getMinDistance(i) * ESTIMATION_COEFFICIENT,
                    //                 // vision.getMinDistance(i) * ESTIMATION_COEFFICIENT,
                    //                 // 5.0));
                
            
        
    //    vision.setReferencePose(getPose()); 2910 thing need look into


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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
      }
      
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
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
        System.out.println("turntoCMD");

        if (shouldEnd) {
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
        return Math.toDegrees(headingController.getPositionError());
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

    public void fieldCentricDrive(double leftX, double leftY, double rightX) {
        this.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(MathUtil.applyDeadband(leftY, 0.1) * Constants.MAX_SPEED_MPS)
                .withVelocityY(MathUtil.applyDeadband(leftX, 0.1) * Constants.MAX_SPEED_MPS)
                .withRotationalRate(MathUtil.applyDeadband(-rightX, 0.1) * Constants.MAX_ANGULAR_RATE));
    }

    public void robotCentricDrive(double leftX, double leftY, double rightX) {
        this.setControl(new SwerveRequest.RobotCentric()
                .withVelocityX(MathUtil.applyDeadband(leftY, 0.1) * Constants.MAX_SPEED_MPS)
                .withVelocityY(MathUtil.applyDeadband(leftX, 0.1) * Constants.MAX_SPEED_MPS)
                .withRotationalRate(MathUtil.applyDeadband(-rightX, 0.1) * Constants.MAX_ANGULAR_RATE));
    }

    // drive robot with field centric angle
    public void driveFieldCentricFacingAngle(double x, double y, double angle, double desiredAngle) {
        FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle().withVelocityX(x).withVelocityY(y)
                .withTargetDirection(Rotation2d.fromDegrees(desiredAngle));
                request.HeadingController = headingController;
        this.setControl(request);
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

    private SwerveDriveKinematics getKinematics() {
     SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = this.getModule(i).getCurrentState();
        }

        SwerveDriveKinematics m_kine = this.m_kinematics;
return m_kine ;
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

        //     if (shouldUseVisionData) {

        // // var visionEst = vision.getEstimatedGlobalPose();
        // // visionEst.ifPresent(
        // //         est -> {
        // //             var estPose = est.estimatedPose.toPose2d();
        // //             // Change our trust in the measurement based on the tags we can see
        // //             var estStdDevs = vision.getEstimationStdDevs(estPose);

        // //             drivetrain.addVisionMeasurement(
        // //                     est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        // //         });
        //     }

        String moduleName = "null";
        for (int i = 0; i < 4; i++) {
            switch (i) {
                case 0:
                    moduleName = "Front Left: ";
                    break;
                case 1:
                    moduleName = "Front Right: ";
                    break;
                case 2:
                    moduleName = "Back Left: ";
                    break;
                case 3:
                    moduleName = "Back Right: ";
                    break;
            }
            Logger.recordOutput(moduleName + "Drive Voltage", this.getModule(i).getDriveMotor().getMotorVoltage().getValueAsDouble());
            Logger.recordOutput(moduleName + "Drive Current", this.getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble());
            Logger.recordOutput(moduleName + "CANCoder Position", this.getModule(i).getCANcoder().getPosition().getValueAsDouble());
            Logger.recordOutput(moduleName + "Steer Voltage", this.getModule(i).getSteerMotor().getMotorVoltage().getValueAsDouble());
            Logger.recordOutput(moduleName + "Steer Current", this.getModule(i).getSteerMotor().getSupplyCurrent().getValueAsDouble());
           
        }
        Logger.recordOutput("Rotation2d", this.getPigeon2().getRotation2d());

    }

}