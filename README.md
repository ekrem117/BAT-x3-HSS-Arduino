# BAT-X3 — Arduino Firmware

TEKNOFEST 2026 Çelik Kubbe Hava Savunma Sistemleri yarışması için
geliştirilen hava savunma taretinin gömülü kontrol yazılımı. Pan, tilt ve
tetik eksenlerini doğrudan Arduino pinlerinden sürer; komutları seri port
üzerinden Python tarafından alır.

Sistemin diğer parçaları:

- Python servisi (görüntü işleme, takip, karar): [ekrem117/bat-x3-hss](https://github.com/ekrem117/bat-x3-hss)
- Operatör arayüzü (C#): [ekrem117/BAT-x3-HSS-GUI](https://github.com/ekrem117/BAT-x3-HSS-GUI)

PCA9685 sürücü kartı kullanımdan kaldırılmıştır. Darbe üretimi artık `Servo.h`
kütüphanesi ile doğrudan dijital pinlerden yapılır; hareket matematiği
PCA9685'li sürümle birebir aynıdır.

---

## Donanım

### Pin atamaları

| Eksen | Pin | Servo | Not |
|---|---|---|---|
| Pan | 3 | DS5160 | Yatay dönüş |
| Tilt | 5 | DS5160 | 1:2 harici dişli ile |
| Tetik 1 | 6 | DS3225 25kg | Aktif |
| Tetik 2 | 9 | DS3225 25kg | **Kod içinde devre dışı** |

### Güç uyarısı

Servo `V+` hattı Arduino'nun 5V pininden **beslenmez**. Harici kaynak
kullanılır:

- Pan / Tilt: 8.4 V
- Tetikler: 6.8 V

Servo sinyal kablosu (sarı/beyaz) yukarıdaki pinlere, GND ise **ortak GND
yıldız noktasına** bağlanır. Ortak GND yoksa sinyal referansı oturmaz ve
servolar titrer.

### Derleme

Arduino IDE ile `arduino_for_joystick_hss.ino` açılır ve yüklenir. Harici
kütüphane gerekmez — `Servo.h` IDE ile birlikte gelir. `.cpp` / `.h`
dosyaları sketch klasöründe durduğu sürece otomatik derlenir.

Seri hız: **115200 baud**.

---

## Seri protokol

Python tarafı tek yönlü komut gönderir, yaklaşık 50 Hz. İki farklı komut
biçimi vardır.

### 1. Bit maskesi (yön tabanlı) — tek bayt

Manuel WASD kontrolü için. Yön belirtir, konum değil.

| Bit | Değer | Tuş | Etki |
|---|---|---|---|
| bit0 | `0x01` | W | Tilt yukarı |
| bit1 | `0x02` | S | Tilt aşağı |
| bit2 | `0x04` | A | Pan sola |
| bit3 | `0x08` | D | Pan sağa |
| bit4 | `0x10` | J | Ateş (kenar algılama) |

Zıt bitler birlikte gelirse (`W`+`S` veya `A`+`D`) o eksen durdurulur.
Bir yön biti basılı kaldığı sürece eksen ilgili limitine doğru hareket
eder; bit kalkınca bulunduğu açıda durur.

### 2. Mutlak açı çerçevesi — 5 bayt

Analog kol ve otonom takip için. Yön değil **konum** belirtir.

```
[0xFB][pan_lo][pan_hi][tilt_lo][tilt_hi]
```

Açı değerleri `int16` little-endian, 0.1 derece biriminde.
Örnek: 47.3° → `473`.

Yarım kalan çerçeve 100 ms içinde tamamlanmazsa iptal edilir; böylece
bağlantı kopması sonraki komutu bozmaz.

### Bağlantı watchdog'u

Python bağlı olduğu sürece en geç 150 ms'de bir bayt gönderir (50 Hz
çerçeve akışı veya heartbeat). **500 ms** boyunca hiç bayt gelmezse
(`COMMAND_TIMEOUT_MS`) eksenler bulundukları açıda durdurulur. Her gelen
bayt — çerçeve veri baytları dahil — watchdog'u besler.

### 3. Özel komutlar — tek bayt

| Bayt | Komut | Etki |
|---|---|---|
| `0xFF` | Acil durdur | Tüm eksenler bulunduğu yerde kilitlenir |
| `0xFE` | Serbest bırak | Acil durdurma mandalını açar |
| `0xFD` | HOME | Tüm eksenleri başlangıç açılarına gönderir |
| `0xFC` | Ateş darbesi | Tetik durum makinesini başlatır |

`0xFC`, bit maskesindeki `0x10` biti yerine ayrı bir komut olarak
tanımlanmıştır. Sebebi: bit maskesi işleyicisi her çağrıldığında pan/tilt
yönünü de değerlendirir ve mutlak açı hedefini bozar.

---

## Mimari

Üç katman, yukarıdan aşağıya tek yönlü bağımlılık:

```
arduino_for_joystick_hss.ino     Protokol çözümleme, mod yönetimi,
                                 tetik durum makinesi, kalibrasyon sabitleri
              |
              v
ServoAxisController              Açı -> pulse dönüşümü, yazılımsal limit,
                                 acil durdurma mandalı
              |
              v
TrajectoryGenerator              Trapez hız profili, ivme limiti,
                                 zaman tabanlı konum hesabı
```

### `TrajectoryGenerator`

Tek eksen için zaman tabanlı, bloklamayan hareket üreteci. Hedef
verildiğinde bir hız profili hesaplar ve her `update()` çağrısında o anki
açıyı döndürür.

İki profil tipi:

- **Trapez** — mesafe yeterince uzunsa: hızlan → sabit hız → yavaşla
- **Üçgen** — mesafe kısaysa tepe hıza çıkılamaz: hızlan → yavaşla

Profil seçimi otomatiktir. `micros()` değeri dışarıdan parametre olarak
verilir, böylece sınıf donanımdan bağımsız test edilebilir.

`isMoving`, `currentAngle`, `setTarget`, `freezeAt` arayüzü sabittir.
İleride S-Curve (jerk limitli) profile geçilirse yalnızca bu sınıfın içi
değişir; üstündeki hiçbir kod etkilenmez.

### `ServoAxisController`

`TrajectoryGenerator`'dan gelen açıyı mikrosaniye darbesine çevirir ve
servoya yazar.

Üç davranış detayı önemlidir:

**Açı → pulse eşlemesi 0–180 aralığı üzerinden yapılır**, eksenin kendi
`MIN/MAX_DEG` sınırları üzerinden değil. Bu sayede eksen limitleri
değiştirildiğinde kalibre edilmiş pulse değerleri kaymaz.

**Yapıcı, açı sınırlarını otomatik sıralar.** Tetik eksenleri REST/FIRE
açılarıyla kuruluyor ve bu açılar ters sırada olabiliyor. Sıralanmazsa
`constrain(x, 180, 0)` her zaman aynı değeri döndürür ve tetik ateş
konumuna hiç gitmez.

**Acil durdurma mandallıdır.** `release()` çağrılana kadar `moveTo()` ve
`update()` hiçbir şey yapmaz. Acil durdurmadan sonra gelen normal
komutlar hareketi yeniden başlatamaz.

`moveToImmediate()` ise profili atlar: hedefi doğrudan yazar ve
trajektoriyi o noktada dondurur. Mutlak açı çerçevesi bu yolu kullanır.

---

## Kalibrasyon değerleri

Aşağıdaki değerler sahada doğrulanmıştır. Değiştirmeden önce mekanik
sınırların nerede olduğunu bilmek gerekir.

Açı sınırları ve HOME değerleri Python tarafındaki `config/settings.py`
(`PAN_*_DEG`, `TILT_*_DEG`, `HOME_*_DEG`) ile **birebir aynı** olmalıdır.
Biri değişirse diğeri de güncellenir.

### Pan (DS5160)

| Parametre | Değer |
|---|---|
| Pulse aralığı | 500 – 1800 µs |
| Açı aralığı | 40° – 100° |
| Maks. hız | 800 °/s |
| Maks. ivme | 1000 °/s² |

### Tilt (DS5160 + 1:2 dişli)

| Parametre | Değer |
|---|---|
| Pulse aralığı | 500 – 1800 µs |
| Açı aralığı | 115° – 150° |
| Maks. hız | 600 °/s |
| Maks. ivme | 800 °/s² |

### Tetik 1 (DS3225)

| Parametre | Değer |
|---|---|
| Pulse aralığı | 600 – 1800 µs |
| Dinlenme açısı | 80° |
| Ateş açısı | 145° |
| Basılı tutma süresi | 250 ms |
| Maks. hız | 3000 °/s |

### HOME açıları

| Eksen | Açı |
|---|---|
| Pan | 70° |
| Tilt | 120° |
| Tetik 1 | 80° (dinlenme) |

### `attach()` aralığı

Tüm servolar `attach(pin, 500, 2500)` ile geniş ortak aralıkta bağlanır.

Bu kasıtlıdır: Arduino `Servo` kütüphanesi `attach()` min/max değerlerini
`int8_t` offset olarak saklar. 1210 gibi dar veya uç değerler verilirse bu
alan taşar ve kütüphane yanlış sınır hesaplar. Gerçek eksen limitleri
zaten `ServoAxisController::writeAngle()` içinde uygulanır; üretilen darbe
her zaman ilgili eksenin kendi pulse aralığındadır.

---

## Güvenlik

**Tetik durum makinesi bloklamaz.** `delay()` kullanılmaz. Dört durum:
`IDLE → FIRING → HOLDING → RETURNING → IDLE`. Ateş sırasında pan/tilt
hareketi ve seri komut alımı kesintisiz sürer.

**Tek atış garantisi.** Bit maskesindeki ateş biti kenar algılama ile
işlenir — tuş basılı kalsa bile tek atış üretilir. Ayrıca `startFire()`
yalnızca `TRIGGER_IDLE` durumundayken hareket başlatır.

**Çerçeve ortasında ateş engellenir.** `0xFC` komutu işlenirken
`frameActive` bayrağı ayrıca kontrol edilir. Kazara ateş en pahalı
hatadır; bu yüzden çift kontrol bırakılmıştır.

**Acil durdurma her şeyi keser.** `0xFF` geldiğinde tüm eksenler
dondurulur, tetik durum makinesi `IDLE`'a alınır ve yarım kalan çerçeve
iptal edilir.

**Bağlantı koparsa taret durur.** Bit maskesi modunda yön komutu hedefi
eksen limiti olarak verir; komut akışı kesilirse eksen limite kadar
dönmeye devam ederdi ve acil durdurma da gönderilemezdi. Watchdog bu
durumda eksenleri bulundukları yerde durdurur, yarım çerçeveyi atar ve
yön durumunu sıfırlar. Bekleyen ateş biti bağlantı dönüşünde yeni atış
üretmez; operatör tetiği bırakıp tekrar basmalıdır. Bu duruş acil
durdurmanın aksine mandallı değildir: komut akışı geri gelince sistem
normal çalışmaya döner, ama duran eksen kendiliğinden harekete geçmez.

---

## Bilinen kısıtlar

**Tetik 2 devre dışı.** İkinci tetiğe ait tüm satırlar yorum içindedir:
pulse aralığı, REST/FIRE açıları, nesne tanımı, `attach()`, `begin()` ve
durum makinesi çağrıları. Ters yönde monte edildiği için açıları Tetik
1'in aynası olacak; pulse aralığı ise bağımsız ve **henüz test
edilmemiştir**. Devreye alırken tilt'te yapıldığı gibi 20–30 µs'lik küçük
adımlarla daraltılmalıdır.

**Mutlak açı modunda profil uygulanmaz.** `moveToImmediate()` hedefi
doğrudan yazar. 50 Hz'de küçük adımlarla gelen komutlarda hareket
akıcıdır, ancak büyük bir sıçrama gelirse eksen ivme limiti olmadan
hareket eder.

**Geri bildirim kanalı yok.** İletişim tek yönlüdür; Arduino yalnızca
açılışta bir hazır mesajı basar. Python tarafı eksenlerin gerçek konumunu
okuyamaz.

**Pozisyon geri beslemesi yok.** Servolar açık çevrimdir. Mekanik
engellenme veya adım kaybı yazılım tarafından algılanamaz.