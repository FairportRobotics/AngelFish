// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.commands.GripperOpenCommand;
import frc.robot.commands.WristCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public final WristCommand wristCommand;

    private final CommandXboxController controller;
    private final CommandXboxController operator;

    private final ArmSubsystem armSubsystem;
    private final GripperSubsystem gripperSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final LightingSubsystem lightingSubsystem;

    private final GripperOpenCommand openGripperCommand;
    private final GripperOpenCommand closeGripperCommand;

    private final ArmCommand substationArmCommand;

    private final ArmCommand lowArmCommand;
    private final ArmCommand midArmCommand;
    private final ArmCommand highArmCommand;

    private final DriveCommand driveCommand;

    private boolean targetingCones;

    private final SendableChooser<Boolean> routeChooser;
    private final SendableChooser<Integer> pickUpCargoChooser;
    private final SendableChooser<Integer> putDownCargoChooser;
    private final SendableChooser<Integer> chargingStationChooser;

    private final PathPoint insideRouteStart;
    private final PathPoint insideRouteEnd;

    private Compressor phCompressor;
    private PneumaticHub ph;

    private final PathPoint outsideRouteStart;
    private final PathPoint outsideRouteEnd;

    private final PathPoint chargingStationStart;

    private final PathPoint[] nodePositions;
    private final PathPoint[] startingCargoPositions;
    private final PathPoint[] chargingStationPos;

    private HashMap<String, Command> eventMap = new HashMap<String, Command>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Controllers
        this.controller = new CommandXboxController(Constants.DRIVER_CONTROLLER);
        this.operator = new CommandXboxController(Constants.OPERATOR_CONTROLLER);

        // Subsystems
        // Initialize Pneumatics Hub
        ph = new PneumaticHub(Constants.PH_CAN_ID);
        phCompressor = ph.makeCompressor();
        phCompressor.enableDigital();
    
        this.armSubsystem = new ArmSubsystem(ph);
        //this.armSubsystem.armMovePosition(2048);
        this.gripperSubsystem = new GripperSubsystem(ph);
        this.driveSubsystem = new DriveSubsystem();

        this.lightingSubsystem = new LightingSubsystem();

        // Commands
        this.openGripperCommand = new GripperOpenCommand(gripperSubsystem, true);
        this.closeGripperCommand = new GripperOpenCommand(gripperSubsystem, false);

        this.substationArmCommand = new ArmCommand(armSubsystem, false, Constants.SUBSTATION_ANGLE);
        this.lowArmCommand = new ArmCommand(armSubsystem, false, Constants.CONE_LOW_ANGLE);
        this.midArmCommand = new ArmCommand(armSubsystem, false, Constants.CONE_MID_ANGLE);
        this.highArmCommand = new ArmCommand(armSubsystem, false, Constants.CONE_HIGH_ANGLE);

        this.wristCommand = new WristCommand(operator, armSubsystem);

        this.driveCommand = new DriveCommand(controller, driveSubsystem);

        // Bindings
        controller.rightBumper().onTrue(Commands.runOnce(()->{driveSubsystem.unlock();}));
        controller.axisGreaterThan(3, 0.75).onTrue(Commands.runOnce(()->{driveSubsystem.lock();}));
        operator.leftBumper().onTrue(openGripperCommand);
        operator.axisGreaterThan(2, 0.75).onTrue(closeGripperCommand);

        operator.rightBumper().onTrue(highArmCommand);
        operator.axisGreaterThan(3, 0.75).onTrue(midArmCommand);
        operator.povDown().onTrue(lowArmCommand);
        operator.povUp().onTrue(substationArmCommand);

        operator.y().onTrue(Commands.runOnce(() -> {
            lightingSubsystem.setConeColor();
            lowArmCommand.setTargetAngle(Constants.CONE_LOW_ANGLE);
            midArmCommand.setTargetAngle(Constants.CONE_MID_ANGLE);
            highArmCommand.setTargetAngle(Constants.CONE_HIGH_ANGLE);
        }));
        operator.x().onTrue(Commands.runOnce(() -> {
            lightingSubsystem.setCubeColor();
            lowArmCommand.setTargetAngle(Constants.CUBE_LOW_ANGLE);
            midArmCommand.setTargetAngle(Constants.CUBE_MID_ANGLE);
            highArmCommand.setTargetAngle(Constants.CUBE_HIGH_ANGLE);
        }));

        // Autonomous options
        this.routeChooser = new SendableChooser<Boolean>();
        this.routeChooser.addOption("Inside", false);
        this.routeChooser.addOption("Outside", true);
        SmartDashboard.putData("Auto Route", routeChooser);

        this.pickUpCargoChooser = new SendableChooser<Integer>();
        this.pickUpCargoChooser.addOption("Position 1", 0);
        this.pickUpCargoChooser.addOption("Position 2", 1);
        this.pickUpCargoChooser.addOption("Position 3", 2);
        this.pickUpCargoChooser.addOption("Position 4", 3);
        SmartDashboard.putData("Cargo Starting Position", pickUpCargoChooser);

        this.putDownCargoChooser = new SendableChooser<Integer>();
        this.putDownCargoChooser.addOption("Row 1", 0);
        this.putDownCargoChooser.addOption("Row 2", 1);
        this.putDownCargoChooser.addOption("Row 3", 2);
        this.putDownCargoChooser.addOption("Row 4", 3);
        this.putDownCargoChooser.addOption("Row 5", 4);
        this.putDownCargoChooser.addOption("Row 6", 5);
        this.putDownCargoChooser.addOption("Row 7", 6);
        this.putDownCargoChooser.addOption("Row 8", 7);
        this.putDownCargoChooser.addOption("Row 9", 8);
        SmartDashboard.putData("Cargo Destination Row", putDownCargoChooser);

        this.chargingStationChooser = new SendableChooser<Integer>();
        this.chargingStationChooser.addOption("Position 1", 0);
        this.chargingStationChooser.addOption("Position 2", 1);
        this.chargingStationChooser.addOption("Position 3", 2);
        SmartDashboard.putData("Charging Station Destination", chargingStationChooser);

        // Autonomous on-the-fly generation points
        this.outsideRouteStart = new PathPoint(new Translation2d(2.1, 4.8), Rotation2d.fromDegrees(0));
        this.outsideRouteEnd = new PathPoint(new Translation2d(5.8, 4.8), Rotation2d.fromDegrees(0));
        this.insideRouteStart = new PathPoint(new Translation2d(2.1, 1.0), Rotation2d.fromDegrees(0));
        this.insideRouteEnd = new PathPoint(new Translation2d(5.8, 1.0), Rotation2d.fromDegrees(0));

        this.chargingStationStart = new PathPoint(new Translation2d(2.6,3.0), Rotation2d.fromDegrees(180));
        
        this.nodePositions = new PathPoint[] {
            new PathPoint(new Translation2d(2,1), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)),
            new PathPoint(new Translation2d(2,1.5), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)),
            new PathPoint(new Translation2d(2,2), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)),
            new PathPoint(new Translation2d(2,2.5), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)),
            new PathPoint(new Translation2d(2,3), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)),
            new PathPoint(new Translation2d(2,3.5), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)),
            new PathPoint(new Translation2d(2,4), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)),
            new PathPoint(new Translation2d(2,4.5), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)),
            new PathPoint(new Translation2d(2,5.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)),
        };
        this.startingCargoPositions = new PathPoint[] {
            new PathPoint(new Translation2d(6.5,4.5), Rotation2d.fromDegrees(0)),
            new PathPoint(new Translation2d(6.5,3.4), Rotation2d.fromDegrees(0)),
            new PathPoint(new Translation2d(6.5,2.1), Rotation2d.fromDegrees(0)),
            new PathPoint(new Translation2d(6.5,1.0), Rotation2d.fromDegrees(0)),
        };
        this.chargingStationPos = new PathPoint[] {
            new PathPoint(new Translation2d(4.0,3.7), Rotation2d.fromDegrees(0)),
            new PathPoint(new Translation2d(4.0,3), Rotation2d.fromDegrees(0)),
            new PathPoint(new Translation2d(4.0,2.2), Rotation2d.fromDegrees(0)),
        };

        // PathPlanner event hashmap
        eventMap.put("openGripper", openGripperCommand);
        eventMap.put("closeGripper", closeGripperCommand);
        eventMap.put("armLow", lowArmCommand);
        eventMap.put("armMid", midArmCommand);
        eventMap.put("armHigh", highArmCommand);

        this.targetingCones = true;
    }

    /**
     * Get the type of cargo that the robot is targeting
     * 
     * @return true if the robot is in cone mode
     */
    public boolean getTargetCargo() {
        return targetingCones;
    }

    public Subsystem getDriveSubsystem() { return this.driveSubsystem; }

    public Command getDriveCommand() { return this.driveCommand; }
    public Command getWristCommand() { return this.wristCommand; }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PathPlannerTrajectory outbound = PathPlanner.generatePath(
            new PathConstraints(4, 3),
            new PathPoint(new Translation2d(2.5, 2), Rotation2d.fromDegrees(0)),
            routeChooser.getSelected() ? insideRouteStart : outsideRouteStart,
            routeChooser.getSelected() ? insideRouteEnd : outsideRouteEnd,
            startingCargoPositions[pickUpCargoChooser.getSelected()]
        );
        PathPlannerTrajectory inbound = PathPlanner.generatePath(
            new PathConstraints(4, 3),
            startingCargoPositions[pickUpCargoChooser.getSelected()],
            routeChooser.getSelected() ? insideRouteEnd : outsideRouteEnd,
            routeChooser.getSelected() ? insideRouteStart : outsideRouteStart,
            nodePositions[putDownCargoChooser.getSelected()]
            
        );
        PathPlannerTrajectory station = PathPlanner.generatePath(
            new PathConstraints(4,3),
            nodePositions[putDownCargoChooser.getSelected()],
            chargingStationStart, 
            chargingStationPos[chargingStationChooser.getSelected()]
        );
        return new SequentialCommandGroup(
            new GripperOpenCommand(gripperSubsystem, true),
            driveSubsystem.followTrajectoryCommand(outbound, true),
            new GripperOpenCommand(gripperSubsystem, false),
            driveSubsystem.followTrajectoryCommand(inbound, false),
            new ArmCommand(armSubsystem, false, Constants.CONE_HIGH_ANGLE),
            new WaitCommand(.5),
            new GripperOpenCommand(gripperSubsystem, true),
            new WaitCommand(2),
            driveSubsystem.followTrajectoryCommand(station, false),
            Commands.runOnce(()->{driveSubsystem.lock();})
        );
    }
}