/*
 * test_comunicacion.ino
 * Prueba inicial de comunicacion ESP32 <-> PC
 *
 * Verifica: puerto serie bidireccional, identificacion del chip,
 * LED integrado y escaneo del bus I2C.
 *
 * Comandos (escribir en el monitor serie + Enter):
 *   ping      -> responde pong
 *   info      -> datos del chip
 *   scan      -> escanea dispositivos I2C
 *   led on    -> enciende LED integrado
 *   led off   -> apaga LED integrado
 *   blink     -> parpadea 5 veces
 */

#include <Wire.h>

const uint8_t  PIN_LED  = 2;      // LED integrado en la mayoria de DevKit V1
const uint8_t  PIN_SDA  = 21;     // I2C por defecto en ESP32
const uint8_t  PIN_SCL  = 22;
const uint32_t BAUDIOS  = 115200;

unsigned long ultimoLatido = 0;
bool ledEncendido = false;

void imprimirInfo() {
  Serial.println(F("--- INFO DEL CHIP ---"));
  uint32_t rev = ESP.getChipRevision();   // core 3.x: major*100+minor
  Serial.printf("Modelo        : %s rev v%lu.%lu\n", ESP.getChipModel(),
                (unsigned long)(rev / 100), (unsigned long)(rev % 100));
  Serial.printf("Nucleos       : %d\n", ESP.getChipCores());
  Serial.printf("Frecuencia CPU: %lu MHz\n", (unsigned long)getCpuFrequencyMhz());
  Serial.printf("Flash         : %lu bytes @ %lu Hz\n",
                (unsigned long)ESP.getFlashChipSize(),
                (unsigned long)ESP.getFlashChipSpeed());
  Serial.printf("RAM libre     : %lu bytes\n", (unsigned long)ESP.getFreeHeap());
  Serial.printf("MAC           : %llX\n", ESP.getEfuseMac());
  Serial.printf("SDK           : %s\n", ESP.getSdkVersion());
  Serial.println(F("---------------------"));
}

void escanearI2C() {
  Serial.println(F("Escaneando bus I2C (SDA=21, SCL=22)..."));
  uint8_t encontrados = 0;
  for (uint8_t dir = 1; dir < 127; dir++) {
    Wire.beginTransmission(dir);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Dispositivo encontrado en 0x%02X\n", dir);
      encontrados++;
    }
  }
  if (encontrados == 0) {
    Serial.println(F("  Ningun dispositivo I2C conectado."));
  } else {
    Serial.printf("  Total: %d dispositivo(s)\n", encontrados);
  }
}

void parpadear(uint8_t veces) {
  for (uint8_t i = 0; i < veces; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(150);
    digitalWrite(PIN_LED, LOW);
    delay(150);
  }
  ledEncendido = false;
}

void procesarComando(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  Serial.printf("[RX] \"%s\"\n", cmd.c_str());

  if (cmd == "ping") {
    Serial.println(F("pong"));
  } else if (cmd == "info") {
    imprimirInfo();
  } else if (cmd == "scan") {
    escanearI2C();
  } else if (cmd == "led on") {
    digitalWrite(PIN_LED, HIGH);
    ledEncendido = true;
    Serial.println(F("LED encendido"));
  } else if (cmd == "led off") {
    digitalWrite(PIN_LED, LOW);
    ledEncendido = false;
    Serial.println(F("LED apagado"));
  } else if (cmd == "blink") {
    Serial.println(F("Parpadeando 5 veces..."));
    parpadear(5);
    Serial.println(F("Listo"));
  } else {
    Serial.println(F("Comando desconocido. Usa: ping | info | scan | led on | led off | blink"));
  }
}

void setup() {
  Serial.begin(BAUDIOS);
  delay(500);                     // margen para que el puerto USB se estabilice

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Wire.begin(PIN_SDA, PIN_SCL);

  Serial.println();
  Serial.println(F("====================================="));
  Serial.println(F(" PID AUTOCLAVE - Test de comunicacion"));
  Serial.println(F("====================================="));
  imprimirInfo();
  escanearI2C();
  Serial.println(F("Comandos: ping | info | scan | led on | led off | blink"));
  Serial.println(F("Listo. Esperando comandos..."));

  parpadear(3);                   // senal visual de arranque correcto
}

void loop() {
  // Latido cada 5 s: confirma que el firmware sigue vivo
  if (millis() - ultimoLatido >= 5000) {
    ultimoLatido = millis();
    Serial.printf("[latido] uptime=%lu s | RAM libre=%lu B | LED=%s\n",
                  millis() / 1000,
                  (unsigned long)ESP.getFreeHeap(),
                  ledEncendido ? "ON" : "OFF");
  }

  // Lectura de comandos entrantes
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    procesarComando(linea);
  }
}
