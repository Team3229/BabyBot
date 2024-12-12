package frc.hawklibraries.drivetrains.tank;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.hawklibraries.utilities.Alliance;
import frc.hawklibraries.utilities.Alliance.AllianceColor;
import frc.hawklibraries.vendorRewrites.wpilib.ChassisSpeeds;

public class TankDrivetrain extends SubsystemBase {
    
    private TankDrivetrainConfig config;

    private DifferentialDriveKinematics kinematics;
    private DifferentialDrivePoseEstimator odometry;
    private AHRS gyro;
    
    private TankModule leftModule;
    private TankModule rightModule;

    private Field2d odometryField;
    private ChassisSpeeds robotCurrentSpeeds;

    private SendableChooser<Command> autoDropdown;

    private Supplier<ChassisSpeeds> inputSpeeds;

    public TankDrivetrain(
        TankDrivetrainConfig config,
        Supplier<ChassisSpeeds> defaultDrivingSpeeds
    ) {

        this.config = config;

        this.leftModule = new TankModule(config.getLeftConfig());
        this.rightModule = new TankModule(config.getRightConfig());
        
        gyro = new AHRS();

        kinematics = new DifferentialDriveKinematics(config.getTrackWidthMeters());

        odometry = new DifferentialDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(),
            leftModule.getDistanceMeters(),
            rightModule.getDistanceMeters(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.7, 0.7, 9999999)
        );

        Alliance.alliancePresentTrigger().onTrue(
            Commands.runOnce(
                () -> {
                        gyro.setAngleAdjustment(
                            (Alliance.getAlliance() == AllianceColor.Red)
                                ? 180
                                : 0
                        );
                    }
                )  
        );

        this.inputSpeeds = defaultDrivingSpeeds;

        setDefaultCommand(driveCommand(inputSpeeds));

        robotCurrentSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        AutoBuilder.configureLTV(
            odometry::getEstimatedPosition,
            this::setOdometryPosition,
            this.robotCurrentSpeeds::getChassisSpeeds,
            (edu.wpi.first.math.kinematics.ChassisSpeeds speeds) -> {
                this.drive(new ChassisSpeeds(speeds));
            },
            0.02,
            new ReplanningConfig(),
            () -> Alliance.getAlliance() == AllianceColor.Red,
            this
        );

        autoDropdown = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Autonomous Selector", autoDropdown);
        
        this.setName("Tank Drivetrain");

    }

    private void drive(ChassisSpeeds speeds) {
        this.robotCurrentSpeeds = speeds;

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(robotCurrentSpeeds.getChassisSpeeds());

        wheelSpeeds.desaturate(config.getMaxSpeed());

        leftModule.setDesiredState(wheelSpeeds.leftMetersPerSecond);
        rightModule.setDesiredState(wheelSpeeds.rightMetersPerSecond);
            
    }

    private Command driveCommand(Supplier<ChassisSpeeds> speeds) {
        Command out = new Command() {
            @Override public void execute() {
                drive(speeds.get());
            }
        };
        out.addRequirements(this);
        return out;
    }

    public Command driveToPose(Double xPoseMeters, Double yPoseMeters, Rotation2d theta, PathConstraints constraints) {
        return AutoBuilder.pathfindToPoseFlipped(
            new Pose2d(
                xPoseMeters == null
                    ? odometry.getEstimatedPosition().getX()
                    : xPoseMeters,
                yPoseMeters == null
                    ? odometry.getEstimatedPosition().getY()
                    : yPoseMeters,
                theta == null
                    ? odometry.getEstimatedPosition().getRotation()
                    : theta
            ),
            constraints
        );
    }

    @Override
    public void periodic() {
        odometry.update(
            this.gyro.getRotation2d(),
            new DifferentialDriveWheelPositions(
                leftModule.getDistanceMeters(),
                rightModule.getDistanceMeters()
            )
        );

        odometryField.setRobotPose(odometry.getEstimatedPosition());
        SmartDashboard.putData(gyro);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    public Field2d getOdometryField() {
        return this.odometryField;
    }

    private void setOdometryPosition(Pose2d pose) {
        odometry.resetPosition(
            gyro.getRotation2d(),
            new DifferentialDriveWheelPositions(
                leftModule.getDistanceMeters(),
                rightModule.getDistanceMeters()
            ),
            pose
        );
    }

    public void addVisionMeasurement(Pose2d position, double timestampSeconds) {
        this.odometry.addVisionMeasurement(position, timestampSeconds);
    }

    public Command getAutonomousCommand() {
        return autoDropdown.getSelected();
    }

}
