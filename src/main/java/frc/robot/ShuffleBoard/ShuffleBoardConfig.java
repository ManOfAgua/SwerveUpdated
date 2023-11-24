package frc.robot.ShuffleBoard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleBoardConfig {
    SendableChooser<String> autonChooser = new SendableChooser<String>();

    public ShuffleBoardConfig() {
        addWidgets();
    }

    public void addWidgets() {
        autonChooser.addOption("Straight path", "StraightPath1");
        SmartDashboard.putData("Auton Chooser", autonChooser);
    }
}