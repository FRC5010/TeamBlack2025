// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.config.json.devices.DeviceConfigReader;
import org.frc5010.common.config.json.devices.DrivetrainConstantsJson;
import org.frc5010.common.constants.RobotConstantsDef;
import org.frc5010.common.drive.swerve.AkitTalonFXSwerveConfig;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.frc5010.common.drive.swerve.SwerveDriveFunctions;
import org.frc5010.common.drive.swerve.akit.AKitTalonFXSwerveDrive;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve.akit.GyroIOPigeon2;
import org.frc5010.common.drive.swerve.akit.GyroIOSim;
import org.frc5010.common.drive.swerve.akit.ModuleIOSim;
import org.frc5010.common.drive.swerve.akit.ModuleIOSpark;
import org.frc5010.common.drive.swerve.akit.ModuleIOSparkTalon;
import org.frc5010.common.drive.swerve.akit.ModuleIOTalonFXReal;
import org.frc5010.common.drive.swerve.akit.ModuleIOTalonFXSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

/** Add your docs here. */
public class AKitSwerveDrivetrainJson implements DrivetrainPropertiesJson {
  public String type = "SparkTalon";
  public DrivetrainConstantsJson constants;
  private Optional<GamePiecesJson> gamePiecesJson = Optional.empty();

  @Override
  public void readDrivetrainConfiguration(GenericRobot robot, File directory) throws IOException {}

  @Override
  public void createDriveTrain(GenericRobot robot) {
    SwerveDriveFunctions driveFunctions;
    AkitTalonFXSwerveConfig config;
    GenericSwerveDrivetrain drivetrain = null;
    config = AkitTalonFXSwerveConfig.builder(constants, drivetrain);

    RobotConfig PP_CONFIG =
        new RobotConfig(
            config.getRobotMass(),
            config.getDriveInertia(),
            new ModuleConfig(
                config.FrontLeft.WheelRadius,
                config.getMaxDriveSpeed().in(MetersPerSecond),
                constants.wheelCOF,
                DeviceConfigReader.getSimulatedMotor(
                        constants.modules.get("frontLeft").driveMotorSetup.motorType, 1)
                    .withReduction(config.FrontLeft.DriveMotorGearRatio),
                config.FrontLeft.SlipCurrent,
                1),
            getModuleTranslations(config));

    SwerveDriveFunctions.mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withBumperSize(config.getBumperFrameWidth(), config.getBumperFrameLength())
            .withRobotMass(config.getRobotMass())
            .withCustomModuleTranslations(getModuleTranslations(config))
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    DeviceConfigReader.getSimulatedMotor(
                        constants.modules.get("frontLeft").driveMotorSetup.motorType, 1),
                    DeviceConfigReader.getSimulatedMotor(
                        constants.modules.get("frontLeft").steerMotorSetup.motorType, 1),
                    config.getDriveGearRatio(),
                    config.getSteerGearRatio(),
                    Volts.of(config.FrontLeft.DriveFrictionVoltage),
                    Volts.of(config.FrontLeft.SteerFrictionVoltage),
                    Meters.of(config.FrontLeft.WheelRadius),
                    KilogramSquareMeters.of(config.FrontLeft.SteerInertia),
                    constants.wheelCOF));

    if (RobotBase.isSimulation()) {
      SwerveDriveFunctions.driveSimulation =
          new SwerveDriveSimulation(
              SwerveDriveFunctions.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
      SimulatedArena.getInstance().addDriveTrainSimulation(SwerveDriveFunctions.driveSimulation);
      if ("TalonFX".equals(type)) {
        driveFunctions =
            new AKitTalonFXSwerveDrive(
                config,
                new GyroIOSim(SwerveDriveFunctions.driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(
                    config, config.FrontLeft, SwerveDriveFunctions.driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(
                    config,
                    config.FrontRight,
                    SwerveDriveFunctions.driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(
                    config, config.BackLeft, SwerveDriveFunctions.driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(
                    config, config.BackRight, SwerveDriveFunctions.driveSimulation.getModules()[3]),
                SwerveDriveFunctions.driveSimulation::setSimulationWorldPose);
      } else {
        driveFunctions =
            new AkitSwerveDrive(
                config,
                new GyroIOSim(SwerveDriveFunctions.driveSimulation.getGyroSimulation()),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                SwerveDriveFunctions.driveSimulation::setSimulationWorldPose);
      }
    } else {
      config.ODOMETRY_FREQUENCY = new CANBus(config.getCanbus()).isNetworkFD() ? 250.0 : 100.0;
      if ("SparkTalon".equals(type)) {
        driveFunctions =
            new AkitSwerveDrive(
                config,
                new GyroIOPigeon2(),
                new ModuleIOSparkTalon(config, config.FrontLeft),
                new ModuleIOSparkTalon(config, config.FrontRight),
                new ModuleIOSparkTalon(config, config.BackLeft),
                new ModuleIOSparkTalon(config, config.BackRight),
                (pose) -> {});
      } else if ("Spark".equals(type)) {
        driveFunctions =
            new AkitSwerveDrive(
                config,
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                (pose) -> {});
      } else if ("TalonFX".equals(type)) {
        driveFunctions =
            new AKitTalonFXSwerveDrive(
                config,
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(config, config.FrontLeft),
                new ModuleIOTalonFXReal(config, config.FrontRight),
                new ModuleIOTalonFXReal(config, config.BackLeft),
                new ModuleIOTalonFXReal(config, config.BackRight),
                (pose) -> {});
      } else {
        throw new IllegalArgumentException("Unknown AkitSwerveDrive type: " + type);
      }
    }
    driveFunctions.setPPRobotConfigSupplier(() -> PP_CONFIG);
    drivetrain =
        new GenericSwerveDrivetrain(
            new LoggedMechanism2d(RobotConstantsDef.robotVisualH, RobotConstantsDef.robotVisualV),
            robot.getDrivetrainConstants(),
            driveFunctions);
    final GenericSwerveDrivetrain dt = drivetrain;
    robot.addSubsystem(ConfigConstants.DRIVETRAIN, drivetrain);
    robot.setPoseSupplier(() -> dt.getPoseEstimator().getCurrentPose());
    robot.setSimulatedPoseSupplier(() -> driveFunctions.getSimPose());
    gamePiecesJson.ifPresent(it -> it.createGamePieces(dt));
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations(AkitTalonFXSwerveConfig config) {
    return new Translation2d[] {
      new Translation2d(config.FrontLeft.LocationX, config.FrontLeft.LocationY),
      new Translation2d(config.FrontRight.LocationX, config.FrontRight.LocationY),
      new Translation2d(config.BackLeft.LocationX, config.BackLeft.LocationY),
      new Translation2d(config.BackRight.LocationX, config.BackRight.LocationY)
    };
  }
}
