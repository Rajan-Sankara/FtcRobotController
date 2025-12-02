package org.firstinspires.ftc.teamcode.DriverCode.FieldCentric;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldCentricInitialization {

    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public IMU imu;

    public void init(HardwareMap hwMap) {
        // Initialize Motors as DcMotorEx for PIDF
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        backRight = hwMap.get(DcMotorEx.class, "backRight");

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reverse left side
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Run using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake when zero power
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Add PIDF coefficients (example values, tune for your robot) ---
        frontLeft.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, 12.0);
        frontRight.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, 12.0);
        backLeft.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, 12.0);
        backRight.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, 12.0);

        // Initialize IMU
        imu = hwMap.get(IMU.class, "imu");

        // If SDK 8.0+, use orientation
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(orientation));
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower  = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower   = forward - strafe + rotate;
        double backRightPower  = forward + strafe - rotate;

        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        // Use setVelocity instead of setPower for PIDF control
        frontLeft.setVelocity((frontLeftPower / maxPower) * 2000);   // ticks/sec target
        frontRight.setVelocity((frontRightPower / maxPower) * 2000);
        backLeft.setVelocity((backLeftPower / maxPower) * 2000);
        backRight.setVelocity((backRightPower / maxPower) * 2000);
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }
}