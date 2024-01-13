package org.firstinspires.ftc.teamcode.photoncore.hardware.servo;


import com.qualcomm.robotcore.hardware.ServoControllerEx;

import java.util.concurrent.CompletableFuture;

public interface PhotonServoController extends ServoControllerEx {
    CompletableFuture<Boolean> isServoPwmEnabledAsync(int servo);

    CompletableFuture<Double> getServoPositionAsync(int servo);
}
