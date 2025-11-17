package frc.robot.swerve;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.util.Units;

import java.io.IOException;

import org.json.simple.parser.ParseException;

public class SwerveConfig {

    public static final double PI2 = 2.0 * Math.PI;

    public static class Constants {
        // BOT SWITCHING
        public double TRACKWIDTH = 30; // 30.0 for MKi
        public double WHEELBASE = 30; // 30.0 for MKi
        public double GEAR_RATIO = 8.14;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

        // DRIVER SETTINGS
        public static int driver = 0;
        public static double transFactor = 1.0;
        public static double rotFactor = .30;

        // AUTON CONSTANTS
        public double XControllerP = 1.6878;
        public double XControllerD = 0;
        public double ThetaControllerP = 0;
        public double ThetaControllerD = 0;
        public RobotConfig autoConfig;

        // BASE CHASSIS CONFIGURATION
        public static final double MAX_VELOCITY = 5.4;
        public static final double MAX_ANGULAR_VELOCITY = Math.PI/6;
        public static final String[] LAYOUT_TITLE = { "Front Left", "Front Right", "Back Left", "Back Right" };
        public static final int[] CHASSIS_ID = { 2, 3, 4, 5 }; // FL, FR, BL, BR
        public static final int PIGEON_ID = 6;

        public Constants(){
            try {
                autoConfig = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                e.printStackTrace();
            }
            SwitchDriver(driver);
        }

        public static void SwitchDriver(int driver){
            switch (driver) {                
                default:
                transFactor = 1.0;
                rotFactor = .5;
                    break;
            }
        }
    }
}