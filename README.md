# AKS - Araç Kontrol Sistemi (Vehicle Control System)

## 📋 Proje Hakkında

AKS (Araç Kontrol Sistemi), elektrikli araçlar için geliştirilmiş gelişmiş bir kontrol sistemidir. Bu sistem, araç kontrol fonksiyonları, telemetri veri iletimi, güvenlik izleme ve izolasyon tespiti gibi kritik özellikleri içerir.

### 🎯 Ana Özellikler

- **Motor Tork Kontrolü**: Hassas motor kontrol algoritmaları
- **Geri Kazanımlı Frenleme**: Enerji geri kazanım optimizasyonu
- **Araç Enerji Yönetimi**: Akıllı enerji dağıtımı
- **Telemetri Sistemi**: LoRa, WiFi ve GPS entegrasyonu
- **Güvenlik İzleme**: İzolasyon direnci izleme ve acil durum yönetimi
- **Araç İçi Haberleşme**: CAN, UART, SPI protokol desteği
- **Arıza Teşhisi**: Kapsamlı hata tespiti ve loglama

## 🏗️ Proje Yapısı

```
AKS/
├── CMakeLists.txt              # Ana CMake yapılandırması
├── platformio.ini             # PlatformIO yapılandırması
├── README.md                   # Bu dosya
├── config/                     # Yapılandırma dosyaları
│   └── STM32F411RETx_FLASH.ld  # Linker script
├── docs/                       # Dokümantasyon
├── drivers/                    # STM32 HAL sürücüleri
├── include/                    # Header dosyaları
│   ├── core/                   # Temel sistem header'ları
│   ├── communication/          # İletişim protokol header'ları
│   ├── sensors/               # Sensör header'ları
│   ├── actuators/             # Aktüatör header'ları
│   ├── safety/                # Güvenlik sistem header'ları
│   ├── utils/                 # Yardımcı fonksiyon header'ları
│   └── hal/                   # HAL katmanı header'ları
├── src/                       # Kaynak kod dosyaları
│   ├── core/                  # Temel sistem implementasyonu
│   ├── communication/         # İletişim protokol implementasyonu
│   ├── sensors/              # Sensör sürücüleri
│   ├── actuators/            # Aktüatör kontrolleri
│   ├── safety/               # Güvenlik sistemi
│   ├── utils/                # Yardımcı fonksiyonlar
│   ├── main.c                # Ana uygulama
│   └── CMakeLists.txt        # Kaynak kod CMake dosyası
├── test/                     # Test dosyaları
│   ├── unit/                 # Birim testleri
│   ├── integration/          # Entegrasyon testleri
│   ├── mocks/                # Mock dosyaları
│   ├── test_runner.c         # Test runner
│   └── CMakeLists.txt        # Test CMake dosyası
└── scripts/                  # Yardımcı scriptler
```

## 🚀 Kurulum ve Derleme

### Gereksinimler

- **CMake 3.20+**
- **ARM GNU Toolchain** (arm-none-eabi-gcc)
- **STM32CubeMX** (opsiyonel)
- **Unity Test Framework** (testler için)

### CMake ile Derleme

#### Host Sistemde Derleme (Testler için)

```bash
mkdir build
cd build
cmake ..
make
```

#### STM32 için Cross-Compile

```bash
mkdir build-arm
cd build-arm
cmake -DCROSS_COMPILE=ON ..
make
```

### PlatformIO ile Derleme

```bash
pio run
pio run --target upload
```

## 🧪 Test Etme

### Tüm Testleri Çalıştırma

```bash
cd build
make run_all_tests
```

### Sadece Birim Testleri

```bash
make run_unit_tests
```

### Sadece Entegrasyon Testleri

```bash
make run_integration_tests
```

### Test Coverage

```bash
cmake -DENABLE_COVERAGE=ON ..
make coverage
```

## 📡 Telemetri Sistemi

### LoRa Konfigürasyonu

- **Frekans**: 868 MHz / 915 MHz
- **Spreading Factor**: 12
- **Bandwidth**: 125 kHz
- **Coding Rate**: 4/5
- **Max Payload**: 64 bytes
- **Şifreleme**: AES-128

### WiFi Konfigürasyonu

- **Protokol**: IEEE 802.11 b/g/n
- **Frekans**: 2.4 GHz
- **Veri Formatı**: JSON/Binary
- **İletişim**: HTTP/MQTT

### GPS Özellikleri

- **Yonga Seti**: NEO-6M
- **Hassasiyet**: 2.5m CEP
- **Güncelleme Hızı**: 1 Hz
- **Desteklenen Sistemler**: GPS, GLONASS

## 🛡️ Güvenlik Sistemi

### İzolasyon İzleme

- **Minimum Direnç**: 50 kΩ
- **Uyarı Seviyesi**: 200 kΩ
- **İyi İzolasyon**: 1 MΩ+
- **Ölçüm Hassasiyeti**: ±2%

### Güvenlik Özellikleri

- Acil durdurma sistemi
- Kapı ve emniyet kemeri kontrolü
- Sıcaklık izleme
- Gerilim/akım koruması
- Hata loglama

## 📊 Performans Özellikleri

### Sistem Özellikleri

- **MCU**: STM32F411RE (84 MHz Cortex-M4)
- **RAM**: 128 KB
- **Flash**: 512 KB
- **CAN Bus**: 500 kbps
- **UART**: 115200 bps
- **ADC**: 12-bit

### Telemetri Performansı

- **Veri İletim Hızı**: 1 saniye aralıklar
- **LoRa Menzil**: 2 km
- **WiFi Veri Hızı**: 250 kbps
- **GPS Fix Süresi**: <30 saniye

## 🔧 Konfigürasyon

### Sistem Konfigürasyonu

```c
aks_config_t system_config = {
    .system_frequency = 84000000,    // 84 MHz
    .can_baudrate = 500000,          // 500 kbps
    .uart_baudrate = 115200,         // 115200 bps
    .adc_resolution = 12,            // 12-bit
    .node_id = 1,                    // Node ID
    .debug_enabled = true            // Debug modu
};
```

### Güvenlik Limitleri

```c
aks_safety_limits_t safety_limits = {
    .min_battery_voltage = 60.0f,           // V
    .max_battery_voltage = 80.0f,           // V
    .max_battery_current = 100.0f,          // A
    .max_battery_temp = 60.0f,              // °C
    .max_motor_temp = 85.0f,                // °C
    .min_isolation_resistance = 50000,      // Ω
    .emergency_timeout = 5000               // ms
};
```

## 📖 API Dokümantasyonu

### Core API

```c
// Sistem başlatma
aks_result_t aks_core_init(const aks_config_t* config);
aks_result_t aks_core_start(void);

// Durum yönetimi
aks_state_t aks_core_get_state(void);
aks_result_t aks_core_set_state(aks_state_t state);
```

### CAN İletişimi

```c
// CAN başlatma
aks_result_t aks_can_init(uint32_t baudrate, const aks_can_filter_t* filters, uint8_t filter_count);

// Veri gönderme/alma
aks_result_t aks_can_transmit(const aks_can_frame_t* frame, uint32_t timeout_ms);
aks_result_t aks_can_receive(aks_can_frame_t* frame, uint32_t timeout_ms);
```

### Telemetri

```c
// Telemetri başlatma
aks_result_t aks_telemetry_init(const aks_lora_config_t* lora_config,
                                const aks_wifi_config_t* wifi_config,
                                const aks_gps_config_t* gps_config);

// Veri gönderme
aks_result_t aks_telemetry_send_packet(const aks_telemetry_packet_t* packet,
                                       aks_telemetry_channel_t channel,
                                       aks_telemetry_format_t format);
```

### Güvenlik

```c
// Güvenlik izleme başlatma
aks_result_t aks_safety_init(const aks_safety_config_t* config);

// İzolasyon ölçümü
aks_result_t aks_safety_measure_isolation(aks_isolation_status_t* status);

// Acil durdurma
aks_result_t aks_safety_emergency_stop(aks_fault_code_t reason);
```

## 🐛 Hata Ayıklama

### Debug Modu

Debug modunu etkinleştirmek için:

```c
aks_core_set_debug(true);
```

### Log Seviyeleri

- **ERROR**: Kritik hatalar
- **WARNING**: Uyarılar
- **INFO**: Bilgi mesajları
- **DEBUG**: Debug mesajları

### Common Issues

1. **CAN Bus Hatası**: Bus terminasyonunu kontrol edin
2. **LoRa Bağlantı Problemi**: Antenna bağlantısını kontrol edin
3. **GPS Fix Alamama**: Açık alanda test edin
4. **İzolasyon Hatası**: Bağlantıları kontrol edin

## 🤝 Katkıda Bulunma

1. Fork yapın
2. Feature branch oluşturun (`git checkout -b feature/amazing-feature`)
3. Değişikliklerinizi commit edin (`git commit -m 'Add amazing feature'`)
4. Branch'inizi push edin (`git push origin feature/amazing-feature`)
5. Pull Request oluşturun

## 📄 Lisans

Bu proje MIT lisansı altında lisanslanmıştır. Detaylar için `LICENSE` dosyasını inceleyiniz.

## 👥 Geliştirici Ekibi

- **AKS Development Team**
- **Version**: 1.0.0
- **Date**: 2025

## 📧 İletişim

Proje ile ilgili sorularınız için issue açabilir veya email gönderebilirsiniz.

---

**Not**: Bu sistem elektrikli araç yarışmaları için geliştirilmiştir ve güvenlik standartlarına uygun olarak tasarlanmıştır.