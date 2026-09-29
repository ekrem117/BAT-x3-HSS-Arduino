#include "ServoAxisController.h"

namespace {
    float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
        return outMin + (x - inMin) * (outMax - outMin) / (inMax - inMin);
    }
}

ServoAxisController::ServoAxisController(Servo &servo,
                                          float minAngleDeg, float maxAngleDeg,
                                          uint16_t minPulseUs, uint16_t maxPulseUs,
                                          float maxVelDegPerSec, float maxAccelDegPerSec2)
    : _servo(servo),
      _traj(maxVelDegPerSec, maxAccelDegPerSec2),
      _minPulseUs(minPulseUs), _maxPulseUs(maxPulseUs),
      _targetAngle(0), _estopped(false) {
    // Aci sinirlarini sirala. Tetikler REST/FIRE acilariyla kuruluyor ve
    // TRIGGER1'de REST(180) > FIRE(0) oldugu icin sira tersine donuyor.
    // Siralanmazsa constrain(x, 180, 0) her zaman ayni degeri dondurur
    // ve tetik hicbir zaman ates konumuna gitmez.
    if (minAngleDeg <= maxAngleDeg) {
        _minAngle = minAngleDeg;
        _maxAngle = maxAngleDeg;
    } else {
        _minAngle = maxAngleDeg;
        _maxAngle = minAngleDeg;
    }
}

void ServoAxisController::begin(float initialAngleDeg) {
    initialAngleDeg = constrain(initialAngleDeg, _minAngle, _maxAngle);
    _targetAngle = initialAngleDeg;
    _traj.freezeAt(initialAngleDeg);
    writeAngle(initialAngleDeg);
}

void ServoAxisController::moveTo(float targetAngleDeg) {
    if (_estopped) return;
    _targetAngle = constrain(targetAngleDeg, _minAngle, _maxAngle);
    _traj.setTarget(_targetAngle, micros());
}

void ServoAxisController::nudge(float deltaDeg) {
    moveTo(_targetAngle + deltaDeg);
}

void ServoAxisController::moveToImmediate(float targetAngleDeg) {
  if (_estopped) return;

  _targetAngle = constrain(targetAngleDeg, _minAngle, _maxAngle);

  _traj.freezeAt(_targetAngle);

  writeAngle(_targetAngle);
}

void ServoAxisController::update() {
    if (_estopped) return;
    float angle = _traj.update(micros());
    writeAngle(angle);
}

void ServoAxisController::emergencyStop() {
    _estopped = true;
    _targetAngle = _traj.currentAngle();
    _traj.freezeAt(_targetAngle);
    writeAngle(_targetAngle);
}

void ServoAxisController::release() {
    _estopped = false;
}

void ServoAxisController::writeAngle(float angleDeg) {
    angleDeg = constrain(angleDeg, _minAngle, _maxAngle);

    // Aci -> pulse esleme aralik 0-180 uzerinden yapilir; boylece eksenin
    // kendi MIN/MAX_DEG sinirlari degisse bile kalibre edilmis pulse
    // degerleri kaymaz (PCA9685'li surumle birebir ayni davranis).
    float pulseUs = mapFloat(angleDeg, 0.0f, 180.0f,
                             (float)_minPulseUs, (float)_maxPulseUs);

    _servo.writeMicroseconds((int)lroundf(pulseUs));
}