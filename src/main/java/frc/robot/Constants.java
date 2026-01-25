package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;


import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.function.Supplier;




public class Constants {

     public static class FieldConstants {
        public static final Distance FIELD_LENGTH = Inches.of(650.12);
        public static final Distance FIELD_WIDTH = Inches.of(316.64);

        public static final Distance ALLIANCE_ZONE = Inches.of(156.06);

        public static final Translation3d HUB_BLUE =
                new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
        public static final Translation3d HUB_RED =
                new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));
        public static final Distance FUNNEL_RADIUS = Inches.of(24);
        public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);
    }

        

  public static class VisionConstants {

    // ---------- FRONT (POSE) CAMERA ----------
    public static final String FRONT_CAMERA_NAME = "frontCamera";
    public static final Transform3d ROBOT_TO_FRONT_CAMERA =
        new Transform3d(
            new Translation3d(0.3, 0.0, 0.2),
            new Rotation3d(0, 0, 0)
        );

    // ---------- SHOOTER (DISTANCE) CAMERA ----------
    public static final String SHOOTER_CAMERA_NAME = "shooterCamera";
    public static final Transform3d ROBOT_TO_SHOOTER_CAMERA =
        new Transform3d(
            new Translation3d(0.15, 0.0, 0.55), // above shooter
            new Rotation3d(0, Units.degreesToRadians(-20), 0) // pitched down
        );

    // ---------- VISION UNCERTAINTY ----------
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
        VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS =
        VecBuilder.fill(0.5, 0.5, 1);

    // ---------- HUB TAGS ----------
    public static final int[] RED_HUB_TAGS  = {2,3,4,5,8,9,10,11};
    public static final int[] BLUE_HUB_TAGS = {18,19,20,21,24,25,26,27};
}

}