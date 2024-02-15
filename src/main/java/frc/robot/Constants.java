package frc.robot;

public class Constants {

        public static final double MAX_SPEED_MPS = 13.7; // used to be 6 meters per second desired top speed (name
                                                         // changed from MAX_SPEED to MAX_SPEED_MPS jan25 kinda sketch)
        public static final double MAX_ANGULAR_RATE = Math.PI * 3; // Half a rotation per second max angular velocity
        
        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 1;

        public static final int WOOD_BOT_LINKAGE_ID = 2;
        public static final int LINKAGE_ID = 14;

        public static final int FLYWHEEL_LEFT_ID = 4; // LEFT & RIGHT CONFIG FOR PRACTICE & COMP BOT, NO LONGER TO BOTTOM
        public static final int FLYWHEEL_RIGHT_ID = 3;

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

        public static final int PIGEON_ID = 13;
}
