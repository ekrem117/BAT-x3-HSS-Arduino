#ifndef SERVO_AXIS_CONTROLLER_H
#define SERVO_AXIS_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>
#include "TrajectoryGenerator.h"

// Servo.h uzerinden tek bir ekseni (Pan, Tilt veya Tetik) surer.
// TrajectoryGenerator'dan gelen ani aciyi mikrosaniye pulse'a cevirir ve
// dogrudan Arduino pininden yazar.
//
// PCA9685AxisController ile ayni public arayuze sahiptir; hareket
// matematigi birebir aynidir. Tek fark darbenin uretildigi yerdir.
class ServoAxisController {
public:
    // NOT: minAngleDeg > maxAngleDeg verilirse degerler otomatik
    // siralanir. Tetik eksenleri REST/FIRE acilariyla kuruluyor ve bu
    // acilar ters sirada olabiliyor; siralanmazsa constrain() ekseni
    // tek noktaya kilitler.
    ServoAxisController(Servo &servo,
                        float minAngleDeg, float maxAngleDeg,
                        uint16_t minPulseUs, uint16_t maxPulseUs,
                        float maxVelDegPerSec, float maxAccelDegPerSec2);

    void begin(float initialAngleDeg);

    // Mutlak hedef acisi belirler (yazilimsal limitler icinde kirpilir).
    void moveTo(float targetAngleDeg);

    // WASD gibi surekli tuslar icin: hedefi mevcut hedefe gore kaydirir.
    void nudge(float deltaDeg);

    //sonradan eklendi
    void moveToImmediate(float targetAngleDeg);

    // Her loop() turunda cagirilmali.
    void update();

    // Acil durdur MANDALLIDIR: release() cagrilana kadar moveTo() ve
    // update() hicbir sey yapmaz. Bu sayede acil durdurdan sonra gelen
    // normal komutlar hareketi yeniden baslatamaz.
    void emergencyStop();
    void release();

    bool isMoving() const { return _traj.isMoving(); }
    float currentAngle() const { return _traj.currentAngle(); }
    float targetAngle() const { return _targetAngle; }
    bool isEmergencyStopped() const { return _estopped; }

private:
    Servo &_servo;
    TrajectoryGenerator _traj;
    float _minAngle, _maxAngle;
    uint16_t _minPulseUs, _maxPulseUs;
    float _targetAngle;
    bool _estopped;

    void writeAngle(float angleDeg);
};

#endif