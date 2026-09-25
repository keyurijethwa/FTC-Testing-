package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem1 {

    private final DcMotorEx shooterL;
    private final DcMotorEx shooterR;

    // ================= TARGET VELOCITIES =================

    private static final double FAST_VELOCITY = 1280;
    private static final double SLOW_VELOCITY = 1260;
    private static final double SHORT_VELOCITY = 1240;
    private static final double LONG_SHOOT = 1440;

    // ================= PIDF =================

    private static final double P = 175;
    private static final double I = 0;
    private static final double D = 5;
    private static final double F = 18;

    // ================= READY TOLERANCE =================

    private static final double READY_TOLERANCE = 40;

    public ShooterSubsystem1(HardwareMap hardwareMap) {

        shooterL = hardwareMap.get(DcMotorEx.class, "SL");
        shooterR = hardwareMap.get(DcMotorEx.class, "SR");

        // Motor directions
        shooterL.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setDirection(DcMotor.Direction.FORWARD);

        // Allow shooter to coast
        shooterL.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooterR.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        // Reset encoders
        shooterL.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        shooterR.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        // Velocity mode
        shooterL.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooterR.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        // PIDF
        shooterL.setVelocityPIDFCoefficients(
                P, I, D, F
        );

        shooterR.setVelocityPIDFCoefficients(
                P, I, D, F
        );
    }

    // =====================================================
    // SHOOT PRESETS
    // =====================================================

    public void shootFast() {
        shootVelocity(FAST_VELOCITY);
    }

    public void shootSlow() {
        shootVelocity(SLOW_VELOCITY);
    }

    public void shortVelocity() {
        shootVelocity(SHORT_VELOCITY);
    }

    public void longShoot() {
        shootVelocity(LONG_SHOOT);
    }

    // =====================================================
    // MAIN VELOCITY CONTROL
    // =====================================================

    public void shootVelocity(double velocity) {

        // Always remain in velocity control
        if (shooterL.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {

            shooterL.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER
            );

            shooterR.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER
            );

            shooterL.setVelocityPIDFCoefficients(
                    P, I, D, F
            );

            shooterR.setVelocityPIDFCoefficients(
                    P, I, D, F
            );
        }

        // Set target velocity
        shooterL.setVelocity(velocity);
        shooterR.setVelocity(velocity);
    }

    // =====================================================
    // STOP
    // =====================================================

    public void stop() {

        shooterL.setVelocity(0);
        shooterR.setVelocity(0);

        shooterL.setPower(0);
        shooterR.setPower(0);
    }

    // =====================================================
    // REVERSE
    // =====================================================

    public void reverse() {

        shooterL.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        shooterR.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        shooterL.setPower(-0.5);
        shooterR.setPower(-0.5);
    }

    // =====================================================
    // AVERAGE VELOCITY
    // =====================================================

    public double getAverageVelocity() {

        double left = Math.abs(
                shooterL.getVelocity()
        );

        double right = Math.abs(
                shooterR.getVelocity()
        );

        return (left + right) / 2.0;
    }

    // =====================================================
    // INDIVIDUAL VELOCITIES
    // =====================================================

    public double getLeftVelocity() {

        return Math.abs(
                shooterL.getVelocity()
        );
    }

    public double getRightVelocity() {

        return Math.abs(
                shooterR.getVelocity()
        );
    }

    // =====================================================
    // READY CHECKS
    // =====================================================

    public boolean readyForFastShot() {

        return getAverageVelocity()
                >= FAST_VELOCITY - READY_TOLERANCE;
    }

    public boolean readyForSlowShot() {

        return getAverageVelocity()
                >= SLOW_VELOCITY - READY_TOLERANCE;
    }

    public boolean readyForShortShot() {

        return getAverageVelocity()
                >= SHORT_VELOCITY - READY_TOLERANCE;
    }

    public boolean readyForLongShot() {

        return getAverageVelocity()
                >= LONG_SHOOT - READY_TOLERANCE;
    }
}