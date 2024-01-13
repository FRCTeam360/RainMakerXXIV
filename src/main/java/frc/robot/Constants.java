package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveTrain;

public class Constants {
        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 1;

        private static final String kCANbusName = "Test 2";

        public static final int INTAKE_ID = 1;
        public static final int SHOOTER_LINKAGE_ID = 2;
        public static final int SHOOTER_LEAD_ID = 3;
        public static final int SHOOTER_FOLLOW_ID = 4;

        private static final int PIGEON_ID = 13;

        public static class SwerveConstants {

                // Front Left
                private static final double kFrontLeftEncoderOffset = -0.37255859375;
                private static final int kFrontLeftSteerMotorId = 10;
                private static final int kFrontLeftEncoderId = 11;
                private static final int kFrontLeftDriveMotorId = 12;
                private static final double kFrontLeftXPosInches = 9.5;
                private static final double kFrontLeftYPosInches = 9.5;

                // Front Right
                private static final double kFrontRightEncoderOffset = 0.431396484375;
                private static final int kFrontRightSteerMotorId = 1;
                private static final int kFrontRightDriveMotorId = 3;
                private static final int kFrontRightEncoderId = 2;
                private static final double kFrontRightXPosInches = 9.5;
                private static final double kFrontRightYPosInches = -9.5;

                // Back Left
                private static final double kBackLeftEncoderOffset = -0.43408203125;
                private static final int kBackLeftSteerMotorId = 7;
                private static final int kBackLeftEncoderId = 8;
                private static final int kBackLeftDriveMotorId = 9;
                private static final double kBackLeftXPosInches = -9.5;
                private static final double kBackLeftYPosInches = 9.5;

                // Back Right
                private static final int kBackRightSteerMotorId = 4;
                private static final int kBackRightEncoderId = 5;
                private static final int kBackRightDriveMotorId = 6;
                private static final double kBackRightEncoderOffset = 0.38720703125;
                private static final double kBackRightXPosInches = -9.5;
                private static final double kBackRightYPosInches = -9.5;

                //for close-loop out type affects pid/ff for steer & drive motors
                private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage; 
                private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage; 

                private static final double kSlipCurrentA = 300.0; // The stator current at which the wheels start to slip;

                public static final double kSpeedAt12VoltsMps = 3.92; // Theoretical free speed (m/s) at 12v applied output
                public static final Slot0Configs steerGains = new Slot0Configs() // The steer motor uses any SwerveModule.SteerRequestType control request with
                        .withKP(100).withKI(0).withKD(0.2) // the output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
                        .withKS(0).withKV(1.5).withKA(0);

                public static final Slot0Configs driveGains = new Slot0Configs() // When using closed-loop control, the drive motor uses the control
                        .withKP(3).withKI(0).withKD(0) // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
                        .withKS(0).withKV(0).withKA(0);

                private static final double kCoupleRatio = 3.5714285714285716; // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;

                private static final double kDriveGearRatio = 8.142857142857142;
                private static final double kSteerGearRatio = 21.428571428571427;
                private static final double kWheelRadiusInches = 2;

                private static final boolean kSteerMotorReversed = true;
                private static final boolean kInvertLeftSide = false;
                private static final boolean kInvertRightSide = true;

                // These are only used for simulation
                private static final double kSteerInertia = 0.00001;
                private static final double kDriveInertia = 0.001;
                // Simulated voltage necessary to overcome friction
                private static final double kSteerFrictionVoltage = 0.25;
                private static final double kDriveFrictionVoltage = 0.25;

                private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                        .withPigeon2Id(PIGEON_ID)
                        .withCANbusName(kCANbusName);

                private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                        .withDriveMotorGearRatio(kDriveGearRatio)
                        .withSteerMotorGearRatio(kSteerGearRatio)
                        .withWheelRadius(kWheelRadiusInches)
                        .withSlipCurrent(kSlipCurrentA)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                        .withSteerInertia(kSteerInertia)
                        .withDriveInertia(kDriveInertia)
                        .withSteerFrictionVoltage(kSteerFrictionVoltage)
                        .withDriveFrictionVoltage(kDriveFrictionVoltage)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withCouplingGearRatio(kCoupleRatio)
                        .withSteerMotorInverted(kSteerMotorReversed);

                private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                        kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
                        kFrontLeftEncoderOffset,
                        Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
                        kInvertLeftSide);
                private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                        kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
                        kFrontRightEncoderOffset,
                        Units.inchesToMeters(kFrontRightXPosInches),
                        Units.inchesToMeters(kFrontRightYPosInches),
                        kInvertRightSide);
                private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                        kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId,
                        kBackLeftEncoderOffset,
                        Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches),
                        kInvertLeftSide);
                private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                        kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
                        kBackRightEncoderOffset,
                        Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
                        kInvertRightSide);

                public static final DriveTrain driveTrain = new DriveTrain(
                        DrivetrainConstants,
                        FrontLeft,
                        FrontRight, BackLeft, BackRight);

        }
}
