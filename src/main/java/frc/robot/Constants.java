package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.swervedrivespecialties.swervelib.ThriftyBotModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public final class Constants {
    public static final class Subsystems {
        public static final class Intake {
            public static double INTAKE_SPEED = 0.35;

            public static int INTAKE_MOTOR_PORT = 4;
        }

        public static final class Drivetrain {

            public static final class FrontLeftModule {
                public static int DRIVE_MOTOR_PORT = 18;
                public static int STEER_MOTOR_PORT = 16;
                public static int STEER_ENCODER_PORT = 10;

                public static double STEER_OFFSET = -Math.toRadians(7.54 + 180);
            }

            public static final class FrontRightModule {
                public static int DRIVE_MOTOR_PORT = 1;
                public static int STEER_MOTOR_PORT = 3;
                public static int STEER_ENCODER_PORT = 3;

                public static double STEER_OFFSET = -Math.toRadians(258.84 + 180);
            }

            public static final class BackLeftModule {
                public static int DRIVE_MOTOR_PORT = 19;
                public static int STEER_MOTOR_PORT = 17;
                public static int STEER_ENCODER_PORT = 13;

                public static double STEER_OFFSET = -Math.toRadians(25 - 35.5 + 180);
            }

            public static final class BackRightModule {
                public static int DRIVE_MOTOR_PORT = 30;
                public static int STEER_MOTOR_PORT = 2;
                public static int STEER_ENCODER_PORT = 2;

                public static double STEER_OFFSET = -Math.toRadians(92.856 + 180);
            }

            public static final double TRACKWIDTH_METERS = 0.7112;
            public static final double TRACKLENGTH_METERS = 0.7112;

            public static SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                    // Front left
                    new Translation2d(TRACKWIDTH_METERS / 2.0, TRACKLENGTH_METERS / 2.0),
                    // Front right
                    new Translation2d(TRACKWIDTH_METERS / 2.0, -TRACKLENGTH_METERS / 2.0),
                    // Back left
                    new Translation2d(-TRACKWIDTH_METERS / 2.0, TRACKLENGTH_METERS / 2.0),
                    // Back right
                    new Translation2d(-TRACKWIDTH_METERS / 2.0, -TRACKLENGTH_METERS / 2.0));

            public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                    ThriftyBotModuleConfigurations.STANDARD.getDriveReduction() *
                    ThriftyBotModuleConfigurations.STANDARD.getWheelDiameter() * Math.PI;

            public static final double MAX_VOLTAGE = 12.0;
        }
    }

    public static final class Commands {
        public static final class FollowPath {
            public static final PIDController X_CONTROLLER = new PIDController(1, 0, 0);
            public static final PIDController Y_CONTROLLER = new PIDController(1, 0, 0);

            public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(1, 0, 0,
                    new TrapezoidProfile.Constraints(Math.PI, Math.PI));
        }
    }

    public static final class RobotContainer {
        public static final class PathPlanner {
            public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2, 2);

            public static final HashMap<String, Command> PATH_EVENTS = new HashMap<>() {
                {
                    put("print", new PrintCommand("Passed print marker"));
                }
            };
        }
    }

    public static final class OI {
        public static final int PRIMARY_CONTROLLER_PORT = 0;
        public static final double DRIVE_SPEED = 1;
    }
}
