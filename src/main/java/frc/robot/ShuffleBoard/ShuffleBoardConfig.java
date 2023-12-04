package frc.robot.ShuffleBoard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleBoardConfig {
    SendableChooser<String> autonChooser = new SendableChooser<String>();

    public ShuffleBoardConfig() {
        addWidgets();
    }

    public void addWidgets() {
        autonChooser.addOption("Straight", "Straight");
        autonChooser.addOption("Cones", "Cones");
        autonChooser.addOption("ConesCurve", "ConesCurve");
        autonChooser.addOption("ConesCurve2", "ConesCurve2");
        
        SmartDashboard.putData("Auton Chooser", autonChooser);
    }
}