#include "TrajectoryGenerator.h"

TrajectoryGenerator::TrajectoryGenerator(float maxVelDegPerSec, float maxAccelDegPerSec2)
    : _maxVel(maxVelDegPerSec), _maxAccel(maxAccelDegPerSec2),
      _startAngle(0), _targetAngle(0), _currentAngle(0),
      _distance(0), _direction(1),
      _tAccel(0), _tCruise(0), _tTotal(0), _peakVel(0),
      _startTimeUs(0), _moving(false) {}

void TrajectoryGenerator::setLimits(float maxVelDegPerSec, float maxAccelDegPerSec2) {
    _maxVel = maxVelDegPerSec;
    _maxAccel = maxAccelDegPerSec2;
}

void TrajectoryGenerator::recomputeProfile() {
    _distance = fabsf(_targetAngle - _startAngle);
    _direction = (_targetAngle >= _startAngle) ? 1.0f : -1.0f;

    if (_distance < 0.0005f) {
        _tAccel = _tCruise = _tTotal = 0;
        _peakVel = 0;
        _moving = false;
        return;
    }

    float tAccFull = _maxVel / _maxAccel;
    float dAccFull = 0.5f * _maxAccel * tAccFull * tAccFull;

    if (2.0f * dAccFull >= _distance) {
        // Üçgen profil: hedefe ulaşmadan tepe hıza çıkılamıyor
        _tAccel = sqrtf(_distance / _maxAccel);
        _peakVel = _maxAccel * _tAccel;
        _tCruise = 0;
    } else {
        // Trapez profil
        _tAccel = tAccFull;
        _peakVel = _maxVel;
        _tCruise = (_distance - 2.0f * dAccFull) / _maxVel;
    }

    _tTotal = 2.0f * _tAccel + _tCruise;
    _moving = true;
}

void TrajectoryGenerator::setTarget(float targetAngleDeg, uint32_t nowUs) {
    _startAngle = _currentAngle;
    _targetAngle = targetAngleDeg;
    _startTimeUs = nowUs;
    recomputeProfile();
}

float TrajectoryGenerator::update(uint32_t nowUs) {
    if (!_moving) {
        return _currentAngle;
    }

    float t = (float)(nowUs - _startTimeUs) / 1000000.0f; // saniye

    float posOffset; // 0..distance arası, başlangıca göre kat edilen mesafe

    if (t >= _tTotal) {
        posOffset = _distance;
        _moving = false;
    } else if (t < _tAccel) {
        posOffset = 0.5f * _maxAccel * t * t;
    } else if (t < _tAccel + _tCruise) {
        float dAcc = 0.5f * _maxAccel * _tAccel * _tAccel;
        posOffset = dAcc + _peakVel * (t - _tAccel);
    } else {
        float tDec = _tTotal - t;
        if (tDec < 0) tDec = 0;
        posOffset = _distance - 0.5f * _maxAccel * tDec * tDec;
    }

    _currentAngle = _startAngle + _direction * posOffset;
    return _currentAngle;
}

void TrajectoryGenerator::freezeAt(float angleDeg) {
    _currentAngle = angleDeg;
    _startAngle = angleDeg;
    _targetAngle = angleDeg;
    _moving = false;
    _tAccel = _tCruise = _tTotal = 0;
}
