package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.ShuffleBoard.ShuffleBoardConfig;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(Constants.ControllerConstants.driver);
    private final Joystick operator = new Joystick(Constants.ControllerConstants.operator);

    /* Drive Controls */
    private final int translationAxis = Constants.DriveConstants.translation.value;
    private final int strafeAxis = Constants.DriveConstants.strafe.value;
    private final int rotationAxis = Constants.DriveConstants.rotation.value;

    /* Driver Buttons */
    //ps4
    private final JoystickButton zeroGyro = new JoystickButton(driver, ControllerConstants.zeroGyro); //O
    private final JoystickButton robotCentric = new JoystickButton(driver, ControllerConstants.robotCentric); //L1
    private final JoystickButton resetModules = new JoystickButton(driver, ControllerConstants.resetModules); //X
    private final JoystickButton slowSwerveon = new JoystickButton(driver, ControllerConstants.slowSwerveon); //Left JoyStick Button
    private final JoystickButton slowSwerveoff = new JoystickButton(driver, ControllerConstants.slowSwerveoff); //Right JoyStick Button

    //xbox
    // private final JoystickButton zeroGyro = new JoystickButton(driver, 4); //Y
    // private final JoystickButton robotCentric = new JoystickButton(driver, 2); //B
    // private final JoystickButton resetModules = new JoystickButton(driver, 3); //X
    // private final JoystickButton slowSwerveon = new JoystickButton(driver, 9); //Left JoyStick Button
    // private final JoystickButton slowSwerveoff = new JoystickButton(driver, 10); //Right JoyStick Button

    /*Operator Controls */
    private final int some1 = Constants.OperateConstants.some1.value;
    private final int some2 = Constants.OperateConstants.some2.value;
    private final int some3 = Constants.OperateConstants.some3.value;

    /*Operator Buttons */
    

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

ShuffleBoardConfig shuffleboardConfig;

    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        shuffleboardConfig = new ShuffleBoardConfig();
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        resetModules.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));

        slowSwerveon.toggleOnTrue(new InstantCommand(() -> s_Swerve.setSlow(true)));
        slowSwerveoff.toggleOnTrue(new InstantCommand(() -> s_Swerve.setSlow(false)));


        /*Operator Buttons */

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        SendableChooser<String> val = (SendableChooser)SmartDashboard.getData("Auton Chooser");
        switch (val.getSelected()) {
            case "Straight":
                return new Straight(s_Swerve);
            case "Cones":
                return new Cones(s_Swerve);
            case "ConesCurve":
                return new ConesCurve(s_Swerve);
            case "ConesCurve2": //ConesCurve that actualy works
                return new ConesCurve2(s_Swerve);
        default:
            return null;
        }
        }
    }


    