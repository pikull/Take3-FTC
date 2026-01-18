package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple; // Add this import

public class Constants {
        public static FollowerConstants followerConstants = new FollowerConstants()
                        .mass(10.52334) // Your robot's weight in kilograms
                        .forwardZeroPowerAcceleration(-44.035) // From the Forward Zero Power Tuner
                        .lateralZeroPowerAcceleration(-82.049)
                        .translationalPIDFCoefficients(new PIDFCoefficients(0.15,0,0.2,0.2))
                        .headingPIDFCoefficients(new PIDFCoefficients(1.1,0,0.02,0.2))
                        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04,-0.001, 0.00001, 0.5,0.04))
                        .centripetalScaling(0.0005);
        // Define your Drivetrain Constants here
        public static MecanumConstants driveConstants = new MecanumConstants()
                        .leftFrontMotorName("leftFront")
                        .leftRearMotorName("leftBack")
                        .rightFrontMotorName("rightFront")
                        .rightRearMotorName("rightBack")
                        .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .xVelocity(78.337)
                        .yVelocity(64.442)

        ;

        public static PathConstraints pathConstraints = new PathConstraints(0.99, 40,
                1,
                1);

        public static PinpointConstants localizerConstants = new PinpointConstants()
                        .forwardPodY(-7.2)
                        .strafePodX(3.43524)
                        .distanceUnit(DistanceUnit.INCH)
                        .hardwareMapName("pinpoint")
                        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        public static Follower createFollower(HardwareMap hardwareMap) {
                return new FollowerBuilder(followerConstants, hardwareMap)
                                .mecanumDrivetrain(driveConstants) // Use the variable we created above
                                .pinpointLocalizer(localizerConstants)
                                .pathConstraints(pathConstraints)
                                .build();
        }
}
