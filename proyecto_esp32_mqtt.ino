/*
  Proyecto: Datalogger ESP32 + MQTT + microSD (sin delay, todo por millis)

  Librerías necesarias (Administrador de bibliotecas):
    - PubSubClient            (Nick O'Leary)
    - ArduinoJson             (Benoit Blanchon, versión 7.x)
    - RTClib                  (Adafruit)
    - OneWire                 (Paul Stoffregen)
    - DallasTemperature       (Miles Burton)
    - Adafruit SSD1306        (+ Adafruit GFX Library, se instala como dependencia)
  SD, SPI, Wire y WiFi ya vienen con el core de ESP32.

  Placa en Arduino IDE: "ESP32 Dev Module"
*/

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <time.h>

// ======================================================================
//                         CONFIGURACIÓN DEL USUARIO
// ======================================================================
// --- Wi-Fi (usar hotspot del celular) ---
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";

// --- MQTT ---
const char*    MQTT_BROKER = "broker.hivemq.com";   // broker público de pruebas
const uint16_t MQTT_PUERTO = 1883;
const char*    MQTT_TOPIC  = ""; // <-- cambia: NúmeroDeCuenta_Nombre

// --- Actuador: se activa por temperatura ---
const float UMBRAL_ACTIVACION_C = 35.0;  // el buzzer suena a partir de este valor
const float HISTERESIS_C        = 1.0;   // se apaga cuando baja de (umbral - histéresis)

// --- Hora ---
const bool  USAR_NTP_PARA_AJUSTAR_RTC = true;   // ajusta el RTC una vez con internet
const long  ZONA_HORARIA_SEG          = -6 * 3600; // Colima: UTC-6

// --- Pines ---
const uint8_t PIN_I2C_SDA   = 21;  // OLED + RTC DS1307
const uint8_t PIN_I2C_SCL   = 22;
const uint8_t PIN_DS18B20   = 4;
const uint8_t PIN_SD_CS     = 32;  // microSD con pines SPI personalizados (según tu PCB)
const uint8_t PIN_SD_SCK    = 33;
const uint8_t PIN_SD_MOSI   = 25;
const uint8_t PIN_SD_MISO   = 26;
const uint8_t PIN_BUZZER    = 17;
const uint8_t PIN_LED_R     = 27;
const uint8_t PIN_LED_G     = 14;
const uint8_t PIN_LED_B     = 13;
const bool    LED_ANODO_COMUN = false;  // true si tu LED RGB es de ánodo común

// --- Pantalla OLED I2C ---
const uint8_t OLED_DIR   = 0x3C;
const uint8_t OLED_ANCHO = 128;
const uint8_t OLED_ALTO  = 64;

// --- Intervalos (ms) ---
const uint32_t INTERVALO_SENSOR_MS   = 5000;
const uint32_t INTERVALO_SD_MS       = 10000;
const uint32_t INTERVALO_MQTT_MS     = 5000;
const uint32_t INTERVALO_PANTALLA_MS = 1000;

const size_t JSON_BUF = 512;

// ======================================================================
//                         DATOS COMPARTIDOS
// ======================================================================
struct EventoActuador {
  bool valido = false;
  char fecha[11] = "";   // DD-MM-YYYY
  char hora[9]   = "";   // HH:MM:SS
};

struct DatosSistema {
  float temperatura = 0.0;
  bool  tempValida  = false;
  bool  actuadorActivo = false;
  EventoActuador ultimaActivacion;
  EventoActuador ultimaDesactivacion;
  bool wifiOk = false;
  bool mqttOk = false;
  bool sdOk   = false;
};

DatosSistema datos;

SPIClass spiSD(VSPI);   // bus SPI con pines personalizados para la microSD

// ======================================================================
//                         CLASE: Temporizador
// ======================================================================
class Temporizador {
 public:
  Temporizador(uint32_t intervaloMs, bool disparoInmediato = false)
      : _intervalo(intervaloMs), _ultimo(0), _primera(disparoInmediato) {}

  bool vencio() {
    uint32_t ahora = millis();
    if (_primera) { _primera = false; _ultimo = ahora; return true; }
    if (ahora - _ultimo >= _intervalo) { _ultimo = ahora; return true; }
    return false;
  }
  void reiniciar() { _ultimo = millis(); _primera = false; }

 private:
  uint32_t _intervalo;
  uint32_t _ultimo;
  bool _primera;
};

// ======================================================================
//                         CLASE: RelojRTC (DS1307)
// ======================================================================
class RelojRTC {
 public:
  RelojRTC() : _tNtp(1000) {}

  bool iniciar() {
    if (!_rtc.begin()) return false;
    if (!_rtc.isrunning()) {
      _rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // hora de compilación
    }
    return true;
  }

  DateTime ahora() { return _rtc.now(); }

  static void fecha(const DateTime& d, char* b) {   // b >= 11 bytes
    snprintf(b, 11, "%02d-%02d-%04d", d.day(), d.month(), d.year());
  }
  static void hora(const DateTime& d, char* b) {    // b >= 9 bytes
    snprintf(b, 9, "%02d:%02d:%02d", d.hour(), d.minute(), d.second());
  }
  static void nombreArchivo(const DateTime& d, char* b, size_t tam) {
    snprintf(b, tam, "/%04d_%02d_%02d.json", d.year(), d.month(), d.day());
  }

  // Ajusta el RTC una sola vez con NTP cuando hay internet (no bloqueante)
  void tarea_ntp(bool wifiOk) {
    if (!USAR_NTP_PARA_AJUSTAR_RTC || _sincronizado || !wifiOk) return;
    if (!_ntpIniciado) {
      configTime(ZONA_HORARIA_SEG, 0, "pool.ntp.org", "time.google.com");
      _ntpIniciado = true;
      _tNtp.reiniciar();
      return;
    }
    if (!_tNtp.vencio()) return;
    time_t t = time(nullptr);
    if (t > 1700000000) {  // ya sincronizó
      struct tm info;
      localtime_r(&t, &info);
      _rtc.adjust(DateTime(info.tm_year + 1900, info.tm_mon + 1, info.tm_mday,
                           info.tm_hour, info.tm_min, info.tm_sec));
      _sincronizado = true;
      Serial.println("[RTC] Hora ajustada con NTP");
    }
  }

 private:
  RTC_DS1307 _rtc;
  Temporizador _tNtp;
  bool _ntpIniciado = false;
  bool _sincronizado = false;
};

// ======================================================================
//                         CLASE: PaqueteJSON
// ======================================================================
class PaqueteJSON {
 public:
  static size_t construir(const DateTime& n, char* salida, size_t tam) {
    JsonDocument doc;
    char f[11], h[9];
    RelojRTC::fecha(n, f);
    RelojRTC::hora(n, h);

    doc["fecha"] = f;
    doc["hora"]  = h;

    JsonObject sensor = doc["sensor"].to<JsonObject>();
    sensor["tipo"]   = "DS18B20";
    sensor["unidad"] = "C";
    if (datos.tempValida) sensor["valor"] = roundf(datos.temperatura * 100.0f) / 100.0f;
    else                  sensor["valor"] = nullptr;

    JsonObject act = doc["actuador"].to<JsonObject>();
    act["nombre"] = "Buzzer";
    act["estado"] = datos.actuadorActivo ? "activado" : "desactivado";
    agregarEvento(act, "activacion",    datos.ultimaActivacion);
    agregarEvento(act, "desactivacion", datos.ultimaDesactivacion);

    return serializeJson(doc, salida, tam);
  }

 private:
  static void agregarEvento(JsonObject act, const char* clave, const EventoActuador& e) {
    if (!e.valido) { act[clave] = nullptr; return; }
    JsonObject o = act[clave].to<JsonObject>();
    o["fecha"] = e.fecha;
    o["hora"]  = e.hora;
  }
};

// ======================================================================
//                         CLASE: SensorTemperatura (DS18B20)
// ======================================================================
class SensorTemperatura {
 public:
  explicit SensorTemperatura(uint8_t pin)
      : _bus(pin), _ds(&_bus), _t(INTERVALO_SENSOR_MS, true) {}

  void iniciar() {
    _ds.begin();
    _ds.setResolution(12);
    _ds.setWaitForConversion(false);  // lectura no bloqueante
  }

  // tarea_sensor: cada 5 s pide la conversión y la lee ~800 ms después
  void tarea_sensor() {
    if (!_esperando) {
      if (_t.vencio()) {
        _ds.requestTemperatures();
        _inicio = millis();
        _esperando = true;
      }
    } else if (millis() - _inicio >= 800) {
      float t = _ds.getTempCByIndex(0);
      _esperando = false;
      if (t == DEVICE_DISCONNECTED_C || t < -55.0f || t > 125.0f) {
        datos.tempValida = false;
        Serial.println("[SENSOR] Lectura invalida (revisa conexion/resistencia 4.7k)");
      } else {
        datos.temperatura = t;
        datos.tempValida = true;
      }
    }
  }

 private:
  OneWire _bus;
  DallasTemperature _ds;
  Temporizador _t;
  uint32_t _inicio = 0;
  bool _esperando = false;
};

// ======================================================================
//                         CLASE: Actuador (buzzer activo)
// ======================================================================
class Actuador {
 public:
  Actuador(uint8_t pin, const char* nombre)
      : _pin(pin), _nombre(nombre), _t(200) {}

  void iniciar() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
  }

  const char* nombre() const { return _nombre; }

  // tarea_actu: lógica de activación (umbral + histéresis)
  void tarea_actu() {
    if (!_t.vencio()) return;
    if (!datos.tempValida) return;   // sin lectura válida mantiene el estado

    bool deseado = datos.actuadorActivo;
    if (!datos.actuadorActivo && datos.temperatura >= UMBRAL_ACTIVACION_C) {
      deseado = true;
    } else if (datos.actuadorActivo &&
               datos.temperatura <= (UMBRAL_ACTIVACION_C - HISTERESIS_C)) {
      deseado = false;
    }

    if (deseado != datos.actuadorActivo) {
      datos.actuadorActivo = deseado;
      digitalWrite(_pin, deseado ? HIGH : LOW);
      _cambioPendiente = true;
    }
  }

  // tarea_actuador: registra la hora exacta de activación / desactivación
  void tarea_actuador(RelojRTC& reloj) {
    if (!_cambioPendiente) return;
    _cambioPendiente = false;

    DateTime n = reloj.ahora();
    EventoActuador& ev = datos.actuadorActivo ? datos.ultimaActivacion
                                              : datos.ultimaDesactivacion;
    ev.valido = true;
    RelojRTC::fecha(n, ev.fecha);
    RelojRTC::hora(n, ev.hora);

    Serial.printf("[ACTUADOR] %s %s el %s a las %s\n", _nombre,
                  datos.actuadorActivo ? "activado" : "desactivado", ev.fecha, ev.hora);
  }

 private:
  uint8_t _pin;
  const char* _nombre;
  Temporizador _t;
  bool _cambioPendiente = false;
};

// ======================================================================
//                         CLASE: PantallaOLED (SSD1306 I2C)
// ======================================================================
class PantallaOLED {
 public:
  PantallaOLED()
      : _oled(OLED_ANCHO, OLED_ALTO, &Wire, -1), _t(INTERVALO_PANTALLA_MS) {}

  bool iniciar() {
    if (!_oled.begin(SSD1306_SWITCHCAPVCC, OLED_DIR)) return false;
    _oled.clearDisplay();
    _oled.setTextColor(SSD1306_WHITE);
    _oled.setTextSize(1);
    _oled.setCursor(0, 0);
    _oled.println("Iniciando...");
    _oled.display();
    return true;
  }

  // tarea_pantalla: fecha, hora, sensor y mensaje del actuador (solo si está activo)
  void tarea_pantalla(RelojRTC& reloj, const char* nombreActuador) {
    if (!_t.vencio()) return;

    DateTime n = reloj.ahora();
    char f[11], h[9], linea[24];
    RelojRTC::fecha(n, f);
    RelojRTC::hora(n, h);

    _oled.clearDisplay();

    _oled.setTextSize(1);
    _oled.setCursor(0, 0);
    _oled.print("Fecha: ");
    _oled.print(f);

    _oled.setTextSize(2);
    _oled.setCursor(0, 14);
    _oled.print(h);

    _oled.setCursor(0, 34);
    if (datos.tempValida) snprintf(linea, sizeof(linea), "%.1f C", datos.temperatura);
    else                  snprintf(linea, sizeof(linea), "--.- C");
    _oled.print(linea);

    if (datos.actuadorActivo) {
      _oled.setTextSize(1);
      _oled.setCursor(0, 56);
      snprintf(linea, sizeof(linea), "%s activado", nombreActuador);
      _oled.print(linea);
    }

    _oled.display();
  }

 private:
  Adafruit_SSD1306 _oled;
  Temporizador _t;
};

// ======================================================================
//                         CLASE: AlmacenamientoSD
// ======================================================================
class AlmacenamientoSD {
 public:
  AlmacenamientoSD() : _t(INTERVALO_SD_MS, true) {}

  bool iniciar() { return montar(); }

  // tarea_sd: cada 10 s añade una línea JSON a /AAAA_MM_DD.json
  void tarea_sd(RelojRTC& reloj) {
    if (!_t.vencio()) return;

    if (!_ok) {              // reintenta montar la tarjeta
      if (!montar()) { Serial.println("[SD] Tarjeta no disponible"); return; }
    }

    DateTime n = reloj.ahora();
    char json[JSON_BUF];
    PaqueteJSON::construir(n, json, sizeof(json));

    char ruta[24];
    RelojRTC::nombreArchivo(n, ruta, sizeof(ruta));

    File archivo = SD.open(ruta, FILE_APPEND);   // crea si no existe, nunca sobrescribe
    if (!archivo) {
      Serial.printf("[SD] No se pudo abrir %s\n", ruta);
      _ok = false;
      datos.sdOk = false;
      return;
    }
    archivo.println(json);   // un JSON por línea
    archivo.close();
    Serial.printf("[SD] Registro guardado en %s\n", ruta);
  }

 private:
  Temporizador _t;
  bool _ok = false;

  // Reinicia el bus SPI y monta la tarjeta (a 1 MHz para tolerar cables/pistas largas)
  bool montar() {
    SD.end();
    spiSD.end();
    spiSD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    _ok = SD.begin(PIN_SD_CS, spiSD, 1000000);
    datos.sdOk = _ok;
    if (_ok) {
      uint8_t tipo = SD.cardType();
      Serial.printf("[SD] Montada. Tipo: %s, tamano: %llu MB\n",
                    tipo == CARD_MMC ? "MMC" : tipo == CARD_SD ? "SDSC" :
                    tipo == CARD_SDHC ? "SDHC" : "desconocido",
                    SD.cardSize() / (1024ULL * 1024ULL));
    }
    return _ok;
  }
};

// ======================================================================
//                         CLASE: ClienteMQTT (Wi-Fi + MQTT)
// ======================================================================
class ClienteMQTT {
 public:
  ClienteMQTT()
      : _mqtt(_wifiCliente),
        _tPublica(INTERVALO_MQTT_MS),
        _tReintentoMqtt(5000),
        _tReintentoWifi(20000) {}

  void iniciar() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    _tReintentoWifi.reiniciar();

    _mqtt.setServer(MQTT_BROKER, MQTT_PUERTO);
    _mqtt.setBufferSize(512);
    _mqtt.setSocketTimeout(2);

    char id[32];
    snprintf(id, sizeof(id), "esp32-%08X", (uint32_t)ESP.getEfuseMac());
    _idCliente = id;
  }

  // tarea_mqtt: mantiene Wi-Fi/MQTT y publica el JSON cada 5 s
  void tarea_mqtt(RelojRTC& reloj) {
    datos.wifiOk = (WiFi.status() == WL_CONNECTED);

    if (!datos.wifiOk) {
      datos.mqttOk = false;
      if (_tReintentoWifi.vencio()) {
        Serial.println("[WIFI] Reintentando conexion...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
      }
      return;
    }

    if (!_wifiAnunciado) {
      Serial.print("[WIFI] Conectado. IP: ");
      Serial.println(WiFi.localIP());
      _wifiAnunciado = true;
    }

    if (!_mqtt.connected()) {
      datos.mqttOk = false;
      if (_tReintentoMqtt.vencio()) {
        if (_mqtt.connect(_idCliente.c_str())) {
          Serial.println("[MQTT] Conectado al broker");
        } else {
          Serial.printf("[MQTT] Fallo de conexion, rc=%d\n", _mqtt.state());
        }
      }
    }

    if (_mqtt.connected()) {
      datos.mqttOk = true;
      _mqtt.loop();

      if (_tPublica.vencio()) {
        DateTime n = reloj.ahora();
        char json[JSON_BUF];
        PaqueteJSON::construir(n, json, sizeof(json));

        if (_mqtt.publish(MQTT_TOPIC, json)) {
          Serial.print("[MQTT] ");
          Serial.println(json);              // cada paquete enviado por la consola serial
        } else {
          Serial.println("[MQTT] Error al publicar");
        }
      }
    } else {
      datos.mqttOk = false;
    }
  }

 private:
  WiFiClient _wifiCliente;
  PubSubClient _mqtt;
  Temporizador _tPublica;
  Temporizador _tReintentoMqtt;
  Temporizador _tReintentoWifi;
  String _idCliente;
  bool _wifiAnunciado = false;
};

// ======================================================================
//                         CLASE: LedEstado (RGB)
// ======================================================================
class LedEstado {
 public:
  LedEstado() : _t(250) {}

  void iniciar() {
    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    color(false, false, false);
  }

  // tarea_LED:
  //   ROJO     -> sin Wi-Fi
  //   AMARILLO -> Wi-Fi conectado, sin MQTT
  //   VERDE    -> Wi-Fi y MQTT conectados
  void tarea_LED() {
    if (!_t.vencio()) return;
    if (!datos.wifiOk)      color(true,  false, false);
    else if (!datos.mqttOk) color(true,  true,  false);
    else                    color(false, true,  false);
  }

 private:
  Temporizador _t;

  static void salida(uint8_t pin, bool encendido) {
    digitalWrite(pin, (encendido != LED_ANODO_COMUN) ? HIGH : LOW);
  }
  void color(bool r, bool g, bool b) {
    salida(PIN_LED_R, r);
    salida(PIN_LED_G, g);
    salida(PIN_LED_B, b);
  }
};

// ======================================================================
//                         OBJETOS GLOBALES
// ======================================================================
RelojRTC          reloj;
SensorTemperatura sensor(PIN_DS18B20);
Actuador          buzzer(PIN_BUZZER, "Buzzer");
PantallaOLED      pantalla;
AlmacenamientoSD  tarjetaSD;
ClienteMQTT       clienteMqtt;
LedEstado         ledEstado;

// ======================================================================
//                         SETUP / LOOP
// ======================================================================
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== Datalogger ESP32 + MQTT ===");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  ledEstado.iniciar();
  buzzer.iniciar();

  if (!pantalla.iniciar()) Serial.println("[OLED] No encontrada (revisa SDA/SCL y direccion 0x3C)");
  if (!reloj.iniciar())    Serial.println("[RTC] DS1307 no encontrado (revisa SDA/SCL y alimentacion)");
  sensor.iniciar();
  if (!tarjetaSD.iniciar()) Serial.println("[SD] No se pudo montar la tarjeta (se reintentara)");

  clienteMqtt.iniciar();
}

void loop() {
  sensor.tarea_sensor();
  buzzer.tarea_actu();
  buzzer.tarea_actuador(reloj);
  pantalla.tarea_pantalla(reloj, buzzer.nombre());
  tarjetaSD.tarea_sd(reloj);
  clienteMqtt.tarea_mqtt(reloj);
  ledEstado.tarea_LED();
  reloj.tarea_ntp(datos.wifiOk);
}
