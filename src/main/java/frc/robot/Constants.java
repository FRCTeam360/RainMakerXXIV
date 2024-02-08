package frc.robot;

import java.io.IOException;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Enumeration;
import java.util.List;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants.RobotType;

public class Constants {
        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 1;

        public static final int SHOOTER_LINKAGE_ID = 2;
        public static final int SHOOTER_TOP_ID = 4;
        public static final int SHOOTER_BOTTOM_ID = 3;
        public static final int INTAKE_ID = 5;

        public static final double MAX_SPEED_MPS = 13.7; // used to be 6 meters per second desired top speed (name changed from MAX_SPEED to MAX_SPEED_MPS jan25 kinda sketch)
        public static final double MAX_ANGULAR_RATE = Math.PI * 3; // Half a rotation per second max angular velocity
        
        // public static final Mode currentMode = Mode.REAL;
        
        // public static RobotType robot = RobotType.WOODBOT;

		// toggle constants between comp bot and practice bot ("beta")
        public static boolean isWoodBot;
        public static boolean isPracticeBot;
        public static boolean isCompBot;
		public static boolean isTestBot;  

        public static boolean isWoodBot(){
                return isWoodBot;
        }
        public static boolean isPracticeBot(){
                return isPracticeBot; 
        }
        public static boolean isCompBot(){
                return isCompBot; 
        }
		public static boolean isTestBot(){
			return isTestBot; 
		}
        
 public static final class MacAddressConstants {
	public static final byte[] TEST_ADDRESS = new byte[]{
			// values are for test -> E4-42-A6-A0-82-05 
			(byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) 0x28, (byte) 0x51, (byte) 0x35
	};
}
public static final class SerialAddressConstants{
        public static final String TEST_SERIAL_ADDRESS = "031b5208";
}
}



//  public static enum RobotType {
//     /* Running on a Wood Robot */
//     WOODBOT, 

//     /* Running on a Practice Robot */
//     PRACTICE,

//     /* Running on a Competition Robot */
//     COMPETITION, 

// 	/* Running on a Test Robot */
// 	TEST
//  }
 
//  public static enum Mode {
//     /** Running on a real robot. */
//     REAL,

//     /** Running a physics simulator. */
//     SIM,

//     /** Replaying from a log file. */
//     REPLAY
//   }


    //public static final int PIGEON_ID = 13;
