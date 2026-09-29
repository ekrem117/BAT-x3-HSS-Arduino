/*
  Celik Kubbe - Servo.h uzerinden WASD + Cift Tetik Manuel Kontrol
  (PCA9685 KALDIRILDI - dogrudan Arduino pinlerinden surulmektedir)

  Bu surum, PCA9685'li son CALISAN kodunuzdaki dogrulanmis kalibrasyon
  degerleri BIREBIR korunarak hazirlanmistir. Hareket matematigi
  (TrajectoryGenerator + aci->pulse donusumu) eskisiyle ozdestir;
  degisen tek sey darbenin PCA9685 yerine Arduino pininden uretilmesidir.

  ONEMLI GUC UYARISI:
  Servo motorlarin V+ (guc) hatti Arduino'nun 5V pininden DEGIL, harici
  8.4V (pan/tilt) ve 6.8V (tetikler) kaynaklardan gelmelidir. Servo
  sinyal kablosu (sari/beyaz) asagidaki pinlere, GND'si ise ORTAK GND
  (yildiz) noktasina baglanir.

  PCA9685/I2C tamamen kaldirildigi icin onceki I2C kilitlenme riski
  (Wire.setWireTimeout korumasi) de ortadan kalkmistir.

  Iletisim protokolu (Python -> Arduino, ~50Hz):

  Tek bayt - yon bitmask'i:
    bit0 (0x01) = W basili  (tilt yukari)
    bit1 (0x02) = S basili  (tilt asagi)
    bit2 (0x04) = A basili  (pan sola)
    bit3 (0x08) = D basili  (pan saga)
    bit4 (0x10) = J basili  (HER IKI tetik birlikte - tek atis, kenar algilama)

  Tek bayt - ozel komutlar:
    0xFF        = ACIL DURDUR (pan+tilt+her iki tetik durur, MANDALLI)
    0xFE        = Acil durdur serbest birak
    0xFD        = HOME (P) - tum eksenleri HOME_*_DEG degerlerine gonderir
    0xFC        = Tek atis darbesi (eksenlere dokunmaz)

  Bes bayt - mutlak aci cercevesi:
    0xFB + [pan_lo][pan_hi][tilt_lo][tilt_hi]
                = int16 little-endian, 0.1 derece birimi (47.3 der -> 473)
                  Analog kol ve otonom takip bu yolu kullanir.

  Baglanti watchdog'u:
    COMMAND_TIMEOUT_MS (500 ms) boyunca hic bayt gelmezse eksenler
    bulunduklari yerde durdurulur. Bkz. loop() sonundaki watchdog blogu.
*/

#include <Servo.h>
#include "TrajectoryGenerator.h"
#include "ServoAxisController.h"

// --- PIN ATAMALARI ---
// Kendi kablolamaniza gore degistirin. Servo.h herhangi bir dijital
// pini kullanabilir (PCA9685'teki gibi ozel kanal numarasi yoktur).
const uint8_t PAN_PIN      = 3;
const uint8_t TILT_PIN     = 5;
const uint8_t TRIGGER1_PIN = 6;
const uint8_t TRIGGER2_PIN = 9;

// --- attach() icin ORTAK GENIS ARALIK ---
// DIKKAT: Arduino Servo kutuphanesi attach(pin, min, max) degerlerini
// int8_t offset olarak saklar; 1210 gibi dar/uc degerler verilirse bu
// alan TASAR ve kutuphane yanlis sinir hesaplar. Bu yuzden tum servolar
// GENIS ortak aralikla attach edilir; gercek eksen limitleri zaten
// ServoAxisController::writeAngle() icinde dogru sekilde uygulanir
// (cikan pulse her zaman ilgili eksenin MIN/MAX_PULSE_US araligindadir).
const uint16_t ATTACH_MIN_US = 500;
const uint16_t ATTACH_MAX_US = 2500;

Servo panServo;
Servo tiltServo;
Servo trigger1Servo;
Servo trigger2Servo;

// --- PAN (DS5160) - son calisan kodunuzdaki dogrulanmis degerler ---
const uint16_t PAN_MIN_PULSE_US = 500;
const uint16_t PAN_MAX_PULSE_US = 1800;
const float PAN_MIN_DEG  = 40.0f;
const float PAN_MAX_DEG  =100.0f;
const float PAN_MAX_VEL_DPS = 800.0f;   // eski: 400
const float PAN_MAX_ACCEL_DPS2 = 1000.0f;   // eski: 500

// --- TILT (DS5160 + 1:2 harici disli) - dogrulanmis degerler ---
const uint16_t TILT_MIN_PULSE_US = 500;  //1000 onceki
const uint16_t TILT_MAX_PULSE_US = 1800;  //1350 onceki
const float TILT_MIN_DEG = 115.0f;  
const float TILT_MAX_DEG = 150.0f; 
const float TILT_MAX_VEL_DPS    = 600.0f;
const float TILT_MAX_ACCEL_DPS2 = 800.0f;

// --- TETIK 1 (DS3225 25kg) - dogrulanmis degerler ---
const uint16_t TRIGGER1_MIN_PULSE_US = 600; //  ates tarafi    //1210 idi
const uint16_t TRIGGER1_MAX_PULSE_US = 1800; // dinlenme tarafi   //1510 idi
const float TRIGGER1_FIRE_DEG = 145.0f;  //75 idi
const float TRIGGER1_REST_DEG = 80.0f; //120 idi

// --- TETIK 2 (DS3225 25kg) - YENI, KARSI YONDE MONTELI ---
// Ters taraftaki silahi tetikledigi icin REST/FIRE acilari TETIK 1'in
// TERSI (mirror) olarak tanimlandi. Pulse araligi ise TETIK 1'den
// BAGIMSIZ ve HENUZ TEST EDILMEMISTIR - asagidaki genis deger yalnizca
// guvenli bir baslangictir. Tilt'te yaptiginiz gibi kucuk adimlarla
// (20-30us) daraltarak kendi guvenli araliginizi bulun.


/*

const uint16_t TRIGGER2_MIN_PULSE_US = 500;  // <-- SIZ TEST EDIP DEGISTIRECEKSINIZ
const uint16_t TRIGGER2_MAX_PULSE_US = 2500; // <-- SIZ TEST EDIP DEGISTIRECEKSINIZ
const float TRIGGER2_REST_DEG = 0.0f;   // TETIK1'in tersi
const float TRIGGER2_FIRE_DEG = 180.0f; // TETIK1'in tersi
*/


const float TRIGGER_MAX_VEL_DPS     = 3000.0f;
const float TRIGGER_MAX_ACCEL_DPS2  = 4000.0f;
const unsigned long TRIGGER_HOLD_MS = 250;





// --- HOME (BASLANGIC / SIFIRLAMA) ACILARI - dogrulanmis degerler ---
const float HOME_PAN_DEG      = 70.0f;
const float HOME_TILT_DEG     = 120.0f;  // = 150.0f
const float HOME_TRIGGER1_DEG = TRIGGER1_REST_DEG;
//const float HOME_TRIGGER2_DEG = TRIGGER2_REST_DEG;








ServoAxisController pan(panServo, PAN_MIN_DEG, PAN_MAX_DEG,
                         PAN_MIN_PULSE_US, PAN_MAX_PULSE_US,
                         PAN_MAX_VEL_DPS, PAN_MAX_ACCEL_DPS2);

ServoAxisController tilt(tiltServo, TILT_MIN_DEG, TILT_MAX_DEG,
                          TILT_MIN_PULSE_US, TILT_MAX_PULSE_US,
                          TILT_MAX_VEL_DPS, TILT_MAX_ACCEL_DPS2);

ServoAxisController trigger1(trigger1Servo, TRIGGER1_REST_DEG, TRIGGER1_FIRE_DEG,
                              TRIGGER1_MIN_PULSE_US, TRIGGER1_MAX_PULSE_US,
                              TRIGGER_MAX_VEL_DPS, TRIGGER_MAX_ACCEL_DPS2);

/*
ServoAxisController trigger2(trigger2Servo, TRIGGER2_REST_DEG, TRIGGER2_FIRE_DEG,
                              TRIGGER2_MIN_PULSE_US, TRIGGER2_MAX_PULSE_US,
                              TRIGGER_MAX_VEL_DPS, TRIGGER_MAX_ACCEL_DPS2);
*/


// Yon durumlarini (state) takip etmek icin degiskenler
int lastPanDir = 0;
int lastTiltDir = 0;
bool lastFireKeyState = false;

//asagisi sonradan eklendi


// --- MUTLAK ACI CERCEVESI (0xFB) ---
// Bicim: [0xFB][pan_lo][pan_hi][tilt_lo][tilt_hi]
// Aci degerleri int16 little-endian, 0.1 derece biriminde.
// Ornek: 47.3 derece -> 473
//
// Bitmask komutlarindan farki: yon degil KONUM belirtir. Python bu
// cerceveyi 50Hz gonderdiginde hedef surekli kayar ve eksenler onu
// TrajectoryGenerator ile puruzsuzce takip eder. Analog kol ve otonom
// takip bu yolu kullanir.
const uint8_t FRAME_HEADER = 0xFB;
const uint8_t FRAME_PAYLOAD_LEN = 4;
const unsigned long FRAME_TIMEOUT_MS = 100;

uint8_t frameBuffer[FRAME_PAYLOAD_LEN];
uint8_t frameIndex = 0;
bool frameActive = false;
unsigned long frameStartMs = 0;

// Ates darbesi komutu. Bitmask'teki 0x10 bitinden ayri tutuldu cunku
// bitmask isleyicisi her cagrildiginda pan/tilt yonunu de degerlendirir
// ve mutlak aci hedefini bozar.
const uint8_t CMD_FIRE_PULSE = 0xFC;

// Aci olcegi: gelen ham deger 10'a bolunerek dereceye cevrilir.
const float ANGLE_SCALE = 10.0f;


// --- BAGLANTI WATCHDOG'U ---
// Python bagli oldugu surece en gec 150 ms'de bir bayt gonderir:
// analog/otonom yolda 50 Hz cerceve akisi, bitmask yolunda ArduinoLink'in
// heartbeat tekrari. 500 ms sessizlik baglantinin koptugu anlamina gelir
// (uc kacirilmis heartbeat).
//
// NEDEN GEREKLI: bitmask modunda yon komutu hedefi EKSEN LIMITI olarak
// verir (moveTo(PAN_MAX_DEG)). Komut akisi kesilirse eksen o limite kadar
// donmeye devam eder ve durdurulamaz; ACIL DURDUR komutu da gonderilemez,
// cunku gonderecek taraf zaten olmustur.
//
// Mutlak aci (0xFB) modunda ayni risk yoktur: moveToImmediate() hedefe
// zaten ulasmis oldugu icin eksen kendiliginden donuk kalir.
const unsigned long COMMAND_TIMEOUT_MS = 500;

unsigned long lastCommandMs = 0;
bool motionHalted = false;


// yukarisi sonradan eklendi


// --- Tetik durum makinesi (non-blocking) - HER IKI tetigi birlikte yonetir ---
enum TriggerState { TRIGGER_IDLE, TRIGGER_FIRING, TRIGGER_HOLDING, TRIGGER_RETURNING };
TriggerState triggerState = TRIGGER_IDLE;
unsigned long triggerHoldStartMs = 0;

void startFire() {
    if (triggerState == TRIGGER_IDLE) {
        trigger1.moveTo(TRIGGER1_FIRE_DEG);
        //trigger2.moveTo(TRIGGER2_FIRE_DEG);
        triggerState = TRIGGER_FIRING;
    }
}

void updateTriggerStateMachine() {
    switch (triggerState) {
        case TRIGGER_IDLE:
            break;

        case TRIGGER_FIRING:
            // Her iki tetik de hareketini bitirdiginde HOLD'a gec
            if (!trigger1.isMoving() ) { //&& !trigger2.isMoving()) {
                triggerHoldStartMs = millis();
                triggerState = TRIGGER_HOLDING;
            }
            break;

        case TRIGGER_HOLDING:
            if (millis() - triggerHoldStartMs >= TRIGGER_HOLD_MS) {
                trigger1.moveTo(TRIGGER1_REST_DEG);
                //trigger2.moveTo(TRIGGER2_REST_DEG);
                triggerState = TRIGGER_RETURNING;
            }
            break;

        case TRIGGER_RETURNING:
            if (!trigger1.isMoving() ) { //&& !trigger2.isMoving()) {
                triggerState = TRIGGER_IDLE;
            }
            break;
    }
}

void goHome() {
    pan.moveTo(HOME_PAN_DEG);
    tilt.moveTo(HOME_TILT_DEG);
    trigger1.moveTo(HOME_TRIGGER1_DEG);
    //trigger2.moveTo(HOME_TRIGGER2_DEG);

    triggerState = TRIGGER_IDLE;
    lastPanDir = 0;
    lastTiltDir = 0;
    lastFireKeyState = false;
}


//yeni fonksiyon


// Tamamlanan cerceveyi isler: iki int16 degeri dereceye cevirip
// eksenlere hedef olarak verir.
void processFrame() {
    int16_t panRaw = (int16_t)(frameBuffer[0] | ((uint16_t)frameBuffer[1] << 8));
    int16_t tiltRaw = (int16_t)(frameBuffer[2] | ((uint16_t)frameBuffer[3] << 8));

    pan.moveToImmediate((float)panRaw / ANGLE_SCALE);
    tilt.moveToImmediate((float)tiltRaw / ANGLE_SCALE);

    // Bitmask durum makinesini sifirla: aksi halde mutlak aci modundan
    // cikip bitmask moduna donuldugunde eski yon hatirlanir ve ilk
    // komut yok sayilir.
    lastPanDir = 0;
    lastTiltDir = 0;
}


//ustteki yeni fonksiyon



void setup() {
    Serial.begin(115200);

    panServo.attach(PAN_PIN, ATTACH_MIN_US, ATTACH_MAX_US);
    tiltServo.attach(TILT_PIN, ATTACH_MIN_US, ATTACH_MAX_US);
    trigger1Servo.attach(TRIGGER1_PIN, ATTACH_MIN_US, ATTACH_MAX_US);
    //trigger2Servo.attach(TRIGGER2_PIN, ATTACH_MIN_US, ATTACH_MAX_US);

    pan.begin(HOME_PAN_DEG);
    tilt.begin(HOME_TILT_DEG);
    trigger1.begin(HOME_TRIGGER1_DEG);
    //trigger2.begin(HOME_TRIGGER2_DEG);

    // Watchdog'u simdiden besle: aksi halde ilk komut gelene kadar
    // gecen sure zaman asimi sayilir ve acilista bosuna tetiklenir.
    lastCommandMs = millis();

    Serial.println(F("Servo.h WASD+Cift Tetik kontrol hazir (PCA9685 yok)."));
}

void loop() {
    if (Serial.available() > 0) {
    // --- Yarim kalan cerceve zaman asimi ---
    // Cerceve ortasinda baglanti koparsa kalan baytlar bir sonraki
    // komutu bozmamalidir.

    if (frameActive && (millis() - frameStartMs > FRAME_TIMEOUT_MS)) {
        frameActive = false;
        frameIndex = 0;
    }

    while (Serial.available() > 0) {
        uint8_t cmd = Serial.read();

        // Her gelen bayt watchdog'u besler. Cerceve veri baytlari da
        // dahildir: onlar da baglantinin canli oldugunun kanitidir.
        lastCommandMs = millis();
        motionHalted = false;

        // Cerceve toplama modundaysak bayti veri olarak biriktir.
        if (frameActive) {
            frameBuffer[frameIndex++] = cmd;
            if (frameIndex >= FRAME_PAYLOAD_LEN) {
                processFrame();
                frameActive = false;
                frameIndex = 0;
            }
            continue;
        }

        if (cmd == FRAME_HEADER) {
            // Cerceve basligi: sonraki 4 bayt veri olarak toplanacak.
            frameActive = true;
            frameIndex = 0;
            frameStartMs = millis();
        } else if (cmd == 0xFF) {
            pan.emergencyStop();
            tilt.emergencyStop();
            trigger1.emergencyStop();
           //trigger2.emergencyStop();
            triggerState = TRIGGER_IDLE;
            frameActive = false;
            frameIndex = 0;
        } else if (cmd == 0xFE) {
            pan.release();
            tilt.release();
            trigger1.release();
            //trigger2.release();
        } else if (cmd == 0xFD) {
            goHome();
        } else if (cmd == CMD_FIRE_PULSE) {
            // Cerceve toplama modunda DEGILSEK ates et. frameActive
            // kontrolu yukarida zaten yapiliyor ama ek guvenlik olarak
            // burada da dogrulaniyor: kazara ates en pahali hatadir.
            if (!frameActive) {
                startFire();
            }
        } else {
            int currentTiltDir = 0;
            int currentPanDir = 0;

            if (cmd & 0x01) currentTiltDir = 1;  // W: Yukari
            if (cmd & 0x02) currentTiltDir = -1;   // S: Asagi
            if ((cmd & 0x01) && (cmd & 0x02)) currentTiltDir = 0;

            if (cmd & 0x04) currentPanDir = 1;   // A: Sola
            if (cmd & 0x08) currentPanDir = -1;  // D: Saga
            if ((cmd & 0x04) && (cmd & 0x08)) currentPanDir = 0;

            bool fireKeyState = (cmd & 0x10) != 0;

            if (currentTiltDir != lastTiltDir) {
                if (currentTiltDir == 1) {
                    tilt.moveTo(TILT_MAX_DEG);
                } else if (currentTiltDir == -1) {
                    tilt.moveTo(TILT_MIN_DEG);
                } else {
                    tilt.moveTo(tilt.currentAngle());
                }
                lastTiltDir = currentTiltDir;
            }

            if (currentPanDir != lastPanDir) {
                if (currentPanDir == 1) {
                    pan.moveTo(PAN_MAX_DEG);
                } else if (currentPanDir == -1) {
                    pan.moveTo(PAN_MIN_DEG);
                } else {
                    pan.moveTo(pan.currentAngle());
                }
                lastPanDir = currentPanDir;
            }

            if (fireKeyState && !lastFireKeyState) {
                startFire();
            }
            lastFireKeyState = fireKeyState;
        }
    }
  }

    // --- Baglanti watchdog'u ---
    // Serial.available() blogunun DISINDA olmak zorundadir: baglanti
    // tamamen sustugunda o blok hic calismaz, yani icine konulan bir
    // kontrol tam ihtiyac duyulan anda devre disi kalir.
    if (!motionHalted && (millis() - lastCommandMs > COMMAND_TIMEOUT_MS)) {
        // Bulundugu yerde dur. emergencyStop() BILINCLI olarak
        // kullanilmadi: o mandallidir ve kisa bir USB titremesinden
        // sonra sistemi 0xFE gelene kadar kilitler. Buradaki durus
        // komut akisi dondugunde kendiliginden cozulur, ancak duran
        // eksen kendiliginden harekete gecmez.
        pan.moveTo(pan.currentAngle());
        tilt.moveTo(tilt.currentAngle());

        // Yarim kalan cerceveyi at. Yukaridaki cerceve zaman asimi da
        // ayni blogun icinde oldugu icin baglanti cerceve ortasinda
        // koparsa asla calismaz; frameActive takili kalir ve baglanti
        // dondugunde ilk dort komut bayti veri sanilip aci olarak
        // yorumlanir.
        frameActive = false;
        frameIndex = 0;

        // Bitmask durum makinesini sifirla ki baglanti dondugunde ilk
        // yon komutu yeniden moveTo() uretsin.
        lastPanDir = 0;
        lastTiltDir = 0;

        // Bekleyen ates biti baglanti donusunde KENAR uretmesin.
        // true birakilinca gelen ilk 0x10'lu bayt yukselen kenar
        // sayilmaz; operator tetigi birakip tekrar basmadan ates olmaz.
        lastFireKeyState = true;

        motionHalted = true;
    }

    pan.update();
    tilt.update();
    trigger1.update();
    //trigger2.update();
    updateTriggerStateMachine();
}

