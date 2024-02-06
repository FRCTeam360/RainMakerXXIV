package frc.robot;


public class Constants {
        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 1;

        public static final int SHOOTER_LINKAGE_ID = 2;
        public static final int SHOOTER_TOP_ID = 4;
        public static final int SHOOTER_BOTTOM_ID = 3;
        public static final int INTAKE_ID = 5;

        public static final double MAX_SPEED_MPS = 13.7; // used to be 6 meters per second desired top speed (name changed from MAX_SPEED to MAX_SPEED_MPS jan25 kinda sketch)
        public static final double MAX_ANGULAR_RATE = Math.PI * 3; // Half a rotation per second max angular velocity
        
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
