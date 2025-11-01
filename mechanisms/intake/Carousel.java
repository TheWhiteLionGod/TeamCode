package org.firstinspires.ftc.teamcode.mechanisms.intake;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HueValues;
import org.firstinspires.ftc.teamcode.ServoPos;
import org.firstinspires.ftc.teamcode.Timings;
import org.firstinspires.ftc.teamcode.hardware.FunctionThread;
import org.firstinspires.ftc.teamcode.hardware.SafeHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.color.ColorSensorHandler;
import org.firstinspires.ftc.teamcode.hardware.motor.ServoHandler;

public class Carousel {
    private final ServoHandler carousel;
    private final Telemetry telemetry;
    private FunctionThread carouselThread;

    public Carousel(SafeHardwareMap safeHardwareMap, Telemetry telemetry) {
        carousel = safeHardwareMap.getServo("Carousel");
        this.telemetry = telemetry;
    }

    public boolean isRunning() {
        return carouselThread == null || carouselThread.isAlive();
    }

    public void startCarousel() {
        if (isRunning()) { return; }
        carouselThread = new FunctionThread(this::spin, () -> {});
        carouselThread.start();
    }

    public void startCarousel(double new_pos) {
        if (isRunning()) { return; }
        carouselThread = new FunctionThread(() -> spin(new_pos), () -> {});
        carouselThread.start();
    }

    public void stopCarousel() {
        if (isRunning()) {
            carouselThread.interrupt();
        }
    }

    // Rotating Carousel to Next Position
    private void spin() {
        double cur_pos = carousel.getPosition();
        if (cur_pos == ServoPos.CAROUSEL_POS_1.getPos()) {
            carousel.setPosition(ServoPos.CAROUSEL_POS_2.getPos());
        }
        else if (cur_pos == ServoPos.CAROUSEL_POS_2.getPos()) {
            carousel.setPosition(ServoPos.CAROUSEL_POS_3.getPos());
        }
        else if (cur_pos == ServoPos.CAROUSEL_POS_3.getPos()) {
            carousel.setPosition(ServoPos.CAROUSEL_POS_1.getPos());
        }
    }

    // Spinning Carousel to Given Position
    private void spin(double new_pos) {
        carousel.setPosition(new_pos);
    }

    // Spinning Carousel for Specific Ball Color
    public void findGreenBall(ColorSensorHandler colorSensor) {
        if (isRunning()) { return; }
        carouselThread = new FunctionThread(() -> findBall(HueValues.GREEN_MIN, HueValues.GREEN_MAX, colorSensor), () -> {});
        carouselThread.start();
    }
    public void findPurpleBall(ColorSensorHandler colorSensor) {
        if (isRunning()) { return; }
        carouselThread = new FunctionThread(() -> findBall(HueValues.PURPLE_MIN, HueValues.PURPLE_MAX, colorSensor), () -> {});
        carouselThread.start();
    }

    private void findBall(HueValues min, HueValues max, ColorSensorHandler colorSensor) throws InterruptedException {
        for (int i = 0; i < 3; i++) {
            float[] hsvValues = colorSensor.hsv();
            if (hsvValues[0] >= min.getHue()
                    && hsvValues[0] <= max.getHue()) {
                return;
            }
            spin();
            Thread.sleep((long) Timings.CAROUSEL_SPIN_TIME.getMilliseconds());
        }
    }
}
