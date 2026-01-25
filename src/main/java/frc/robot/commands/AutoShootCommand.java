package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.Constants.VisionConstants;

public class AutoShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;

    public AutoShootCommand(
        ShooterSubsystem shooter,
        VisionSubsystem vision
    ) {
        this.shooter = shooter;
        this.vision = vision;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        vision.getDistanceToTagMeters(
            VisionConstants.BLUE_HUB_TAGS
        ).ifPresent(distance -> {
            shooter.shootForDistance(distance);
        });
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
