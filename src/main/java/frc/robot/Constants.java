package frc.robot;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants.RobotType;

public class Constants {
        public static class WoodBotConstants {
                public static final int WOOD_BOT_LINKAGE_ID = 2;
        }
        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 1;

        public static final int LINKAGE_ID = 14;

        public static final int FLYWHEEL_LEFT_ID = 3; // LEFT & RIGHT CONFIG FOR PRACTICE & COMP BOT, NO LONGER TO BOTTOM
        public static final int FLYWHEEL_RIGHT_ID = 4;

        public static final int INTAKE_ID = 5;

        public static final int CLIMBER_LEFT_ID = 6; // ARBITRARY LMAO
        public static final int CLIMBER_RIGHT_ID = 7;

        public static final int INTAKE_HIGH_SENSOR_PORT = 0;
        public static final int INTAKE_SIDE_SENSOR_PORT = 1;
        public static final int LINKAGE_ZERO_BUTTON_PORT = 2;
        public static final int LINKAGE_BRAKE_TOGGLE_BUTTON_PORT = 3;

        public static final Mode currentMode = Mode.REAL;

        public static enum Mode {
                /** Running on a real robot. */
                REAL,

                /** Running a physics simulator. */
                SIM,

                /** Replaying from a log file. */
                REPLAY
        }

        public static final double MAX_SPEED_MPS = 13.7; // used to be 6 meters per second desired top speed (name
                                                         // changed from MAX_SPEED to MAX_SPEED_MPS jan25 kinda sketch)
        public static final double MAX_ANGULAR_RATE = Math.PI * 3; // Half a rotation per second max angular velocity
        public static final int PIGEON_ID = 13;

        public static boolean isWoodBot() {
                if (getRobotType() == Constants.RobotType.WOODBOT) {
                        return true;
                } else {
                        return false;
                }
        }

        public static boolean isPracticeBot() {
                if (getRobotType() == Constants.RobotType.PRACTICE) {
                        return true;
                } else {
                        return false;
                }
        }

        public static boolean isCompBot() {
                if (getRobotType() == Constants.RobotType.COMPETITION) {
                        return true;
                } else {
                        return false;
                }
        }

        public static boolean isTestBot() {
                if (getRobotType() == Constants.RobotType.TEST) {
                        return true;
                } else {
                        return false;
                }
        }

        public static enum RobotType {
                /** Running on a real robot. */
                REAL,

                /** Running a physics simulator. */
                SIM,

                /** Replaying from a log file. */
                REPLAY,

                /* Running on a Wood Robot */
                WOODBOT,

                /* Running on a Practice Robot */
                PRACTICE,

                /* Running on a Competition Robot */
                COMPETITION,

                /* Running on a Test Robot */
                TEST
        }

        public static RobotType getRobotType() {
                String serialAddress = HALUtil.getSerialNumber();
                SmartDashboard.putString("serial address", serialAddress);
                SmartDashboard.putString("Test Serial Address", SerialAddressConstants.TEST_SERIAL_ADDRESS);

                if (serialAddress.equals(SerialAddressConstants.TEST_SERIAL_ADDRESS)) {
                        return Constants.RobotType.TEST;

                } else if (serialAddress.equals(SerialAddressConstants.WOOD_SERIAL_ADDRESS)) {
                        return Constants.RobotType.WOODBOT;
                }
                else if(serialAddress.equals(SerialAddressConstants.PRACTICE_SERIAL_ADDRESS)){
                        return Constants.RobotType.PRACTICE;
                }
                else if(serialAddress.equals(SerialAddressConstants.COMP_SERIAL_ADDRESS)){
                        return Constants.RobotType.COMPETITION;

                }
        }

        public static final class SerialAddressConstants {
                public static final String TEST_SERIAL_ADDRESS = "031b5208";
                public static final String WOOD_SERIAL_ADDRESS = "03064db8";
                public static final String PRACTICE_SERIAL_ADDRESS = "03126d42";
                public static final String COMP_SERIAL_ADDRESS = "03260AD5";
        }
        }
}
