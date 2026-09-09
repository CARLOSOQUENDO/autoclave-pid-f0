/*
 * 01_validacion_pt100.ino
 * PID Autoclave All American 75X - Paso 1 del orden de desarrollo
 *
 * Objetivo: validar la cadena de medicion PT100 -> MAX31865 -> ESP32 antes de
 * tocar cualquier elemento de potencia. Verifica Rref, modo 3 hilos, filtro de
 * 60 Hz y flags de falla, y permite contrastar contra hielo (0 C) y agua en
 * ebullicion.
 *
 * NO energiza el SSR. GPIO25 se fuerza a LOW como primera instruccion de setup().
 *
 * PRESUPUESTO DE TIEMPO
 * ---------------------
 * Adafruit_MAX31865::readRTD() bloquea ~75 ms por conversion (10 ms de
 * estabilizacion de bias + 65 ms de conversion), y lo hace siempre: forzar
 * autoConvert(true) no sirve porque readRTD() dispara el modo one-shot de todos
 * modos, y readRegister16() es privado, asi que no se puede puentear.
 *
 * Consecuencia: cada conversion cuesta 75 ms. Por eso aqui se hace UNA sola
 * conversion por muestra y de ella se derivan resistencia y temperatura via
 * calculateTemperature(), que es solo aritmetica. Llamar a readRTD() y luego a
 * temperature() cuesta el doble sin ganar nada.
 *
 *   mediana de 3 -> 225 ms  (22 % de un periodo de 1 s)  <- elegido
 *   mediana de 5 -> 375 ms  (38 % de un periodo de 1 s)
 *
 * En ESP32 delay() llama a vTaskDelay(), asi que ese bloqueo cede la CPU: en la
 * arquitectura FreeRTOS de la seccion 8 la tarea de UI corre sin problema
 * durante esas esperas. Aun asi conviene dejar margen holgado en la tarea de
 * control.
 *
 * Comandos por monitor serie (115200 baudios):
 *   diag    -> volcado de diagnostico y comprobacion de Rref
 *   crudo   -> lecturas sin filtrar, 20 muestras
 *   hielo   -> referencia punto de hielo (0.00 C), 30 s de estadistica
 *   agua    -> referencia ebullicion (segun ALTITUD_M), 30 s de estadistica
 *   ruido   -> desviacion estandar de la medicion, evalua el filtrado
 *   tiempo  -> mide la latencia real de una conversion
 *   fallas  -> lee y decodifica el registro de fallas
 *   help    -> lista de comandos
 */

#include <Adafruit_MAX31865.h>
#include <math.h>

// ---------------------------------------------------------------------------
// CONFIGURACION
// ---------------------------------------------------------------------------

// -- Pines (seccion 3 del documento de contexto) --
const uint8_t PIN_SSR      = 25;    // salida de potencia: se fuerza LOW, no se usa aqui
const uint8_t PIN_CS       = 5;     // strapping pin; si hay arranques erraticos mover a 32/33
// SPI por hardware (VSPI): SCK=18, MISO=19 (<-SDO), MOSI=23 (->SDI)

// -- MAX31865 / PT100 --
const float RREF           = 430.0; // OHM. Modulo PT100 = 430. Si el modulo trae 4300 es PT1000.
const float RNOMINAL       = 100.0; // OHM de la PT100 a 0 C
const bool  FILTRO_50HZ    = false; // false = 60 Hz (red de 120 VAC / 60 Hz)

// -- Referencias de calibracion --
const float ALTITUD_M      = 1495.0; // Medellin, Parque Explora (~1495 m)
const float TEMP_HIELO_C   = 0.00;   // punto de hielo fundente en equilibrio

// -- Filtrado (seccion 6: mediana + IIR simple) --
const uint8_t N_MEDIANA    = 3;     // impar. Ver PRESUPUESTO DE TIEMPO arriba.
const float   ALFA_IIR     = 0.20;  // 0 = muy suave/lento, 1 = sin filtrar

// -- Rango valido de operacion (seccion 7.2) --
const float T_MIN_VALIDA   = 0.0;
const float T_MAX_VALIDA   = 150.0;

const uint32_t PERIODO_MS  = 1000;  // tiempo de muestreo fijo (seccion 6: 500 ms - 1 s)

Adafruit_MAX31865 termo(PIN_CS);    // SPI por hardware

float tFiltrada = NAN;              // estado del filtro IIR
bool  iirIniciado = false;
unsigned long ultimaMuestra = 0;

// ---------------------------------------------------------------------------
// UTILIDADES
// ---------------------------------------------------------------------------

// Punto de ebullicion del agua corregido por altitud (aprox. barometrica)
float puntoEbullicion() {
  float presion_kPa = 101.325 * pow(1.0 - 2.25577e-5 * ALTITUD_M, 5.25588);
  // Clausius-Clapeyron alrededor de 100 C
  return 1.0 / (1.0 / 373.15 - 8.314 / 40660.0 * log(presion_kPa / 101.325)) - 273.15;
}

inline float rawAOhm(uint16_t raw) { return (raw / 32768.0f) * RREF; }

inline float rawAGrados(uint16_t raw) {
  return termo.calculateTemperature(raw, RNOMINAL, RREF);
}

int comparaU16(const void *a, const void *b) {
  int d = (int)*(const uint16_t *)a - (int)*(const uint16_t *)b;
  return (d > 0) - (d < 0);
}

// Mediana de N_MEDIANA conversiones. Coste: N_MEDIANA * ~75 ms.
uint16_t leerRawMediana() {
  uint16_t m[N_MEDIANA];
  for (uint8_t i = 0; i < N_MEDIANA; i++) m[i] = termo.readRTD();
  qsort(m, N_MEDIANA, sizeof(uint16_t), comparaU16);
  return m[N_MEDIANA / 2];
}

// Aplica el IIR sobre un valor ya medido (no dispara conversiones nuevas)
float aplicarIIR(float x) {
  if (isnan(x)) return tFiltrada;
  if (!iirIniciado) { tFiltrada = x; iirIniciado = true; }
  else              { tFiltrada = ALFA_IIR * x + (1.0f - ALFA_IIR) * tFiltrada; }
  return tFiltrada;
}

// Devuelve true si hay falla; imprime el detalle
bool revisarFallas(bool silencioso) {
  uint8_t f = termo.readFault();
  if (!f) {
    if (!silencioso) Serial.println(F("Fallas: ninguna."));
    return false;
  }
  Serial.printf("FALLA (registro 0x%02X):\n", f);
  if (f & MAX31865_FAULT_HIGHTHRESH) Serial.println(F("  - RTD por encima del umbral alto"));
  if (f & MAX31865_FAULT_LOWTHRESH)  Serial.println(F("  - RTD por debajo del umbral bajo"));
  if (f & MAX31865_FAULT_REFINLOW)   Serial.println(F("  - REFIN- > 0.85 x Vbias"));
  if (f & MAX31865_FAULT_REFINHIGH)  Serial.println(F("  - REFIN- < 0.85 x Vbias (FORCE- abierto)"));
  if (f & MAX31865_FAULT_RTDINLOW)   Serial.println(F("  - RTDIN- < 0.85 x Vbias (FORCE- abierto)"));
  if (f & MAX31865_FAULT_OVUV)       Serial.println(F("  - Sobretension / subtension"));
  termo.clearFault();
  return true;
}

// ---------------------------------------------------------------------------
// COMANDOS
// ---------------------------------------------------------------------------

void cmdDiag() {
  Serial.println(F("\n=== DIAGNOSTICO DE LA CADENA DE MEDICION ==="));
  Serial.printf("Rref configurada : %.1f ohm\n", RREF);
  Serial.printf("R nominal PT100  : %.1f ohm\n", RNOMINAL);
  Serial.printf("Filtro           : %s\n", FILTRO_50HZ ? "50 Hz" : "60 Hz");
  Serial.println(F("Modo             : 3 hilos"));
  Serial.printf("Mediana / periodo: %u muestras (~%u ms) / %lu ms\n",
                N_MEDIANA, N_MEDIANA * 75, (unsigned long)PERIODO_MS);
  Serial.printf("Altitud          : %.0f m -> ebullicion a %.2f C\n",
                ALTITUD_M, puntoEbullicion());

  uint16_t raw = termo.readRTD();          // UNA sola conversion
  float ohm = rawAOhm(raw);
  float t   = rawAGrados(raw);

  Serial.println(F("--- Lectura instantanea ---"));
  Serial.printf("RTD crudo (15 bit): %u\n", raw);
  Serial.printf("Ratio             : %.6f\n", raw / 32768.0);
  Serial.printf("Resistencia       : %.3f ohm\n", ohm);
  Serial.printf("Temperatura       : %.2f C\n", t);

  Serial.println(F("--- Comprobaciones ---"));
  if (raw == 0) {
    Serial.println(F("[X] RTD crudo = 0. Revisar cableado SPI (SDO->GPIO19, SDI->GPIO23) y CS."));
  } else if (raw >= 32767) {
    Serial.println(F("[X] RTD crudo saturado. Sonda en corto o Rref incorrecta."));
  } else {
    Serial.println(F("[OK] Conversion ADC dentro de rango."));
  }

  // Una PT100 entre 0 y 150 C debe medir entre 100 y 158 ohm
  if (ohm > 90.0 && ohm < 170.0) {
    Serial.println(F("[OK] Resistencia coherente con PT100 (100-158 ohm en 0-150 C)."));
  } else if (ohm > 900.0 && ohm < 1700.0) {
    Serial.println(F("[X] Resistencia ~10x alta: la sonda es PT1000, no PT100."));
    Serial.println(F("    Corregir RNOMINAL=1000.0 y usar modulo con Rref=4300."));
  } else if (ohm > 9.0 && ohm < 20.0) {
    Serial.println(F("[X] Resistencia ~10x baja: el modulo trae Rref=4300 (version PT1000)."));
    Serial.println(F("    Medir la resistencia de referencia en la placa y ajustar RREF."));
  } else {
    Serial.printf("[?] Resistencia fuera de lo esperado (%.1f ohm). Revisar sonda y Rref.\n", ohm);
  }

  if (isnan(t)) {
    Serial.println(F("[X] Temperatura NaN. Sonda desconectada o falla activa."));
  } else if (t < T_MIN_VALIDA || t > T_MAX_VALIDA) {
    Serial.printf("[X] Temperatura fuera del rango valido (%.1f-%.1f C).\n",
                  T_MIN_VALIDA, T_MAX_VALIDA);
  } else {
    Serial.println(F("[OK] Temperatura dentro del rango valido."));
  }

  revisarFallas(false);
  Serial.println(F("============================================\n"));
}

void cmdCrudo() {
  Serial.println(F("\n--- 20 lecturas sin filtrar ---"));
  Serial.println(F("  #   RTD    ohm      C"));
  for (uint8_t i = 0; i < 20; i++) {
    uint16_t raw = termo.readRTD();
    Serial.printf("%3u  %5u  %7.3f  %7.3f\n", i + 1, raw, rawAOhm(raw), rawAGrados(raw));
  }
  Serial.println();
}

void cmdTiempo() {
  Serial.println(F("\n--- LATENCIA DE CONVERSION ---"));
  const uint8_t N = 10;

  unsigned long t0 = micros();
  for (uint8_t i = 0; i < N; i++) termo.readRTD();
  unsigned long dt1 = micros() - t0;

  t0 = micros();
  for (uint8_t i = 0; i < N; i++) leerRawMediana();
  unsigned long dt2 = micros() - t0;

  Serial.printf("readRTD()        : %.2f ms por llamada\n", dt1 / 1000.0 / N);
  Serial.printf("leerRawMediana() : %.2f ms por llamada (mediana de %u)\n",
                dt2 / 1000.0 / N, N_MEDIANA);
  Serial.printf("Carga del periodo: %.1f %% de %lu ms\n",
                (dt2 / 1000.0 / N) * 100.0 / PERIODO_MS, (unsigned long)PERIODO_MS);
  Serial.println();
}

// Estadistica sobre una ventana de tiempo, contra una referencia conocida
void estadistica(const char *etiqueta, float referencia, uint16_t segundos) {
  Serial.printf("\n--- REFERENCIA %s: %.2f C ---\n", etiqueta, referencia);
  Serial.printf("Midiendo %u s. Deje que la lectura se estabilice antes de juzgar.\n", segundos);

  uint32_t n = 0;
  double suma = 0, sumaSq = 0;
  float minT = 1e9, maxT = -1e9;
  unsigned long fin = millis() + (unsigned long)segundos * 1000UL;
  unsigned long proxImpresion = millis();
  float t = NAN;

  while ((long)(millis() - fin) < 0) {
    t = rawAGrados(leerRawMediana());
    if (!isnan(t)) {
      n++; suma += t; sumaSq += (double)t * t;
      if (t < minT) minT = t;
      if (t > maxT) maxT = t;
    }
    if ((long)(millis() - proxImpresion) >= 0) {
      proxImpresion += 3000;
      Serial.printf("  quedan %2lus  T=%7.3f C\n", (fin - millis()) / 1000, t);
    }
  }

  if (n == 0) { Serial.println(F("Sin lecturas validas.")); return; }

  double media = suma / n;
  double var = (sumaSq / n) - (media * media);
  double sd = var > 0 ? sqrt(var) : 0.0;
  double error = media - referencia;

  Serial.println(F("--- RESULTADO ---"));
  Serial.printf("Muestras     : %lu\n", (unsigned long)n);
  Serial.printf("Media        : %.3f C\n", media);
  Serial.printf("Desv. tipica : %.3f C\n", sd);
  Serial.printf("Min / Max    : %.3f / %.3f C  (span %.3f)\n", minT, maxT, maxT - minT);
  Serial.printf("Referencia   : %.3f C\n", referencia);
  Serial.printf("ERROR        : %+.3f C\n", error);

  if (fabs(error) <= 0.5)      Serial.println(F("[OK] Dentro de +/-0.5 C. Cadena de medicion valida."));
  else if (fabs(error) <= 1.5) Serial.println(F("[~] Error moderado. Aceptable, considerar offset de calibracion."));
  else                         Serial.println(F("[X] Error alto. Revisar modo 3 hilos, Rref y contacto de la sonda."));
  Serial.println();
}

void cmdRuido() {
  Serial.println(F("\n--- RUIDO: 60 muestras, cruda vs. filtrada ---"));
  // Una sola conversion por muestra: la misma lectura alimenta ambas series.
  double sC = 0, sqC = 0, sF = 0, sqF = 0;
  uint16_t n = 0;
  for (uint16_t i = 0; i < 60; i++) {
    float c = rawAGrados(termo.readRTD());
    if (isnan(c)) continue;
    float f = aplicarIIR(c);
    n++;
    sC += c; sqC += (double)c * c;
    sF += f; sqF += (double)f * f;
  }
  if (n < 2) { Serial.println(F("Sin lecturas validas.")); return; }
  double mC = sC / n, mF = sF / n;
  double sdC = sqrt(fmax(0.0, sqC / n - mC * mC));
  double sdF = sqrt(fmax(0.0, sqF / n - mF * mF));
  Serial.printf("Cruda    : media %.3f C, sd %.4f C\n", mC, sdC);
  Serial.printf("Filtrada : media %.3f C, sd %.4f C\n", mF, sdF);
  if (sdF > 1e-6) Serial.printf("Reduccion de ruido: %.1fx\n", sdC / sdF);
  Serial.println();
}

void ayuda() {
  Serial.println(F("Comandos: diag | crudo | hielo | agua | ruido | tiempo | fallas | help"));
}

void procesarComando(String cmd) {
  cmd.trim(); cmd.toLowerCase();
  if (!cmd.length()) return;

  if      (cmd == "diag")   cmdDiag();
  else if (cmd == "crudo")  cmdCrudo();
  else if (cmd == "hielo")  estadistica("HIELO", TEMP_HIELO_C, 30);
  else if (cmd == "agua")   estadistica("EBULLICION", puntoEbullicion(), 30);
  else if (cmd == "ruido")  cmdRuido();
  else if (cmd == "tiempo") cmdTiempo();
  else if (cmd == "fallas") revisarFallas(false);
  else if (cmd == "help")   ayuda();
  else { Serial.printf("Comando desconocido: \"%s\"\n", cmd.c_str()); ayuda(); }
}

// ---------------------------------------------------------------------------

void setup() {
  // SEGURIDAD (seccion 7.2): estado inicial seguro ANTES de cualquier otra cosa.
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, LOW);

  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n================================================"));
  Serial.println(F(" PID AUTOCLAVE 75X - Paso 1: validacion PT100"));
  Serial.println(F(" SSR forzado a LOW. No se energiza nada."));
  Serial.println(F("================================================"));

  termo.begin(MAX31865_3WIRE);
  termo.enable50Hz(FILTRO_50HZ);   // false -> 60 Hz, red de 120 VAC
  termo.clearFault();
  delay(100);

  cmdDiag();
  ayuda();
  Serial.printf("\nLectura continua cada %lu ms:\n", (unsigned long)PERIODO_MS);
}

void loop() {
  if (millis() - ultimaMuestra >= PERIODO_MS) {
    ultimaMuestra = millis();

    uint16_t raw = leerRawMediana();       // N_MEDIANA conversiones
    float ohm   = rawAOhm(raw);
    float cruda = rawAGrados(raw);
    float filt  = aplicarIIR(cruda);

    Serial.printf("T=%7.3f C (filt %7.3f) | %7.3f ohm | RTD %5u",
                  cruda, filt, ohm, raw);

    if (isnan(cruda)) {
      Serial.print(F("  <- NaN"));
    } else if (cruda < T_MIN_VALIDA || cruda > T_MAX_VALIDA) {
      Serial.print(F("  <- FUERA DE RANGO"));
    }
    Serial.println();

    revisarFallas(true);
  }

  if (Serial.available()) {
    procesarComando(Serial.readStringUntil('\n'));
  }
}
