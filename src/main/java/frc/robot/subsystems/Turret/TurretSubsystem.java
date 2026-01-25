package frc.robot.subsystems.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TurretSubsystem extends SubsystemBase {

    // ------------------------------------------------
    // CONSTANTS
    // ------------------------------------------------

    // Offset of shooter relative to robot center (meters)
    private static final Translation2d SHOOTER_OFFSET =
        new Translation2d(0.35, 0.0);

    // ------------------------------------------------
    // DEPENDENCIES
    // ------------------------------------------------

    private final VisionSubsystem vision;
    private final SwerveSubsystem swerve;

    // ------------------------------------------------
    // STATE
    // ------------------------------------------------

    private boolean hubTrackingEnabled = false;
    private double manualOmega = 0.0;

    private Rotation2d desiredFieldHeading = new Rotation2d();
    private double distanceToHubMeters = 0.0;

    // ------------------------------------------------
    // CONTROLLERS
    // ------------------------------------------------

    private final PIDController headingPID =
        new PIDController(6.0, 0.0, 0.25);

    // ------------------------------------------------
    // CONSTRUCTOR
    // ------------------------------------------------

    public TurretSubsystem(
        VisionSubsystem vision,
        SwerveSubsystem swerve
    ) {
        this.vision = vision;
        this.swerve = swerve;

        headingPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    // ------------------------------------------------
    // PERIODIC
    // ------------------------------------------------

    @Override
    public void periodic() {
        updateTargeting();
        logToAdvantageScope();
    }

    // ------------------------------------------------
    // TARGETING MATH
    // ------------------------------------------------

    private void updateTargeting() {

        Translation3d hub =
            DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Blue
                    ? Constants.FieldConstants.HUB_BLUE
                    : Constants.FieldConstants.HUB_RED;

        Pose2d robotPose = swerve.getPose();
        Rotation2d robotYaw = robotPose.getRotation();

        Translation2d shooterFieldPos =
            robotPose.getTranslation()
                .plus(SHOOTER_OFFSET.rotateBy(robotYaw));

        Translation2d toHub =
            hub.toTranslation2d().minus(shooterFieldPos);

        desiredFieldHeading =
            new Rotation2d(
                Math.atan2(toHub.getY(), toHub.getX())
            );

        distanceToHubMeters = toHub.getNorm();
    }

    // ------------------------------------------------
    // CONTROL API (USED BY ROBOTCONTAINER)
    // ------------------------------------------------

    /** Enable automatic hub tracking */
    public void enableHubTracking() {
        hubTrackingEnabled = true;
    }

    /** Disable tracking, allow manual control */
    public void disableHubTracking() {
        hubTrackingEnabled = false;
        manualOmega = 0.0;
    }

    /** Manual rotation input (rad/s equivalent) */
    public void manualRotate(double omega) {
    manualOmega = omega;

    if (Math.abs(omega) > 0.05) {
        hubTrackingEnabled = false;
    }
}


    /** Output omega for swerve rotation */
    public double getDesiredRobotOmega() {
        if (hubTrackingEnabled) {
            return headingPID.calculate(
                swerve.getPose().getRotation().getRadians(),
                desiredFieldHeading.getRadians()
            );
        }
        return manualOmega;
    }

    public Rotation2d getDesiredRobotHeading() {
        return desiredFieldHeading;
    }

    public double getDistanceToHubMeters() {
        return distanceToHubMeters;
    }

    // ------------------------------------------------
    // LOGGING (ADVANTAGESCOPE)
    // ------------------------------------------------

    private void logToAdvantageScope() {

        SmartDashboard.putBoolean(
            "Turret/HubTrackingEnabled",
            hubTrackingEnabled
        );

        SmartDashboard.putNumber(
            "Targeting/DistanceMeters",
            distanceToHubMeters
        );

        SmartDashboard.putNumber(
            "Targeting/DesiredHeadingDeg",
            desiredFieldHeading.getDegrees()
        );

        SmartDashboard.putNumber(
            "Targeting/RobotYawDeg",
            swerve.getPose().getRotation().getDegrees()
        );

        SmartDashboard.putNumber(
            "Targeting/HeadingErrorDeg",
            desiredFieldHeading
                .minus(swerve.getPose().getRotation())
                .getDegrees()
        );
    }
}
