#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <Arduino.h>

// Zaman tabanlı (non-blocking), trapezoidal hız profiliyle
// yumuşak başlangıç/bitiş, hız limiti ve ivme limiti sağlayan
// tek eksenli hareket üreteci.
//
// İleride S-Curve (jerk-limited) yükseltmesi için bu sınıfın
// public arayüzü (setTarget/update/isMoving/currentAngle/freezeAt)
// korunarak iç hesaplama değiştirilebilir; AxisController ve
// üstündeki hiçbir kod etkilenmez.
class TrajectoryGenerator {
public:
    // maxVelDegPerSec   : maksimum açısal hız (derece/saniye)
    // maxAccelDegPerSec2: maksimum açısal ivme (derece/saniye^2)
    TrajectoryGenerator(float maxVelDegPerSec, float maxAccelDegPerSec2);

    void setLimits(float maxVelDegPerSec, float maxAccelDegPerSec2);

    // Yeni bir hedef açı. Başlangıç noktası = o anki komut edilen açı.
    // nowUs: micros() değeri (çağıran taraf sağlar, böylece test edilebilir)
    void setTarget(float targetAngleDeg, uint32_t nowUs);

    // Anlık açının yeniden hesaplanması. Her loop() turunda çağırılmalı.
    // Dönen değer: o anki komut edilmesi gereken servo açısı (derece)
    float update(uint32_t nowUs);

    bool isMoving() const { return _moving; }
    float currentAngle() const { return _currentAngle; }

    // Acil durum: hareketi olduğu yerde dondurur, profili iptal eder.
    void freezeAt(float angleDeg);

private:
    float _maxVel;
    float _maxAccel;

    float _startAngle;
    float _targetAngle;
    float _currentAngle;

    float _distance;   // hedefe olan mutlak mesafe (derece)
    float _direction;  // +1 / -1

    float _tAccel;    // hızlanma süresi (s)
    float _tCruise;   // sabit hız süresi (s)
    float _tTotal;    // toplam süre (s)
    float _peakVel;   // ulaşılan tepe hız (deg/s)

    uint32_t _startTimeUs;
    bool _moving;

    void recomputeProfile();
};

#endif
