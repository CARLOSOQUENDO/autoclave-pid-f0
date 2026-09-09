/*
 * 02_pwm_lento.ino
 * PID Autoclave All American 75X - Paso 2 del orden de desarrollo
 *
 * Etapa de salida: PWM lento (time-proportional) para SSR zero-cross.
 *
 * *** PROBAR SOLO CON UNA LAMPARA INCANDESCENTE, NUNCA CON LA RESISTENCIA ***
 *
 * ARQUITECTURA (seccion 8)
 * ------------------------
 * La lectura del MAX31865 bloquea ~225 ms (mediana de 3 x 75 ms, ver §6.1). Si
 * eso ocurriera dentro del mismo bucle que genera el PWM, la ventana se
 * desfasaria hasta 225 ms sobre 2000 ms = 11 % de error de duty. Por eso:
 *
 *   tareaSalida  (prio 3, cada 5 ms)  -> conmuta el SSR, nunca se bloquea
 *   tareaSensor  (prio 2, cada 1 s)   -> lee la PT100, puede bloquearse
 *   loop()       (prio 1)             -> consola serie
 *
 * SEGURIDAD (seccion 7.2)
 * -----------------------
 *  - digitalWrite(PIN_SSR, LOW) es la primera instruccion de setup().
 *  - La salida arranca DESARMADA. Hace falta 'armar' explicito.
 *  - Hombre muerto: si no llega ningun comando en 10 min, duty -> 0.
 *  - Watchdog de 8 s sobre la tarea de salida.
 *  - Cualquier falla del sensor o lectura fuera de rango desarma la salida.
 *
 * Comandos (115200 baudios):
 *   armar          -> habilita la salida (obligatorio antes de nada)
 *   desarmar       -> deshabilita y pone duty a 0
 *   d <0-100>      -> fija el duty en %
 *   off / on       -> atajos para d 0 / d 100
 *   v <ms>         -> cambia la ventana (por defecto 2000 ms)
 *   estado         -> duty comandado, duty real medido, temperatura
 *   secuencia      -> barrido automatico 0-25-50-75-100-0, 3 ventanas c/u
 *   emi            -> compara ruido de la PT100 con SSR quieto vs conmutando
 *   help
 */

#include <Adafruit_MAX31865.h>
#include <esp_task_wdt.h>
#include <math.h>

// ---------------------------------------------------------------------------
// CONFIGURACION
// ---------------------------------------------------------------------------

const uint8_t PIN_SSR   = 25;
const uint8_t PIN_CS    = 5;

const float RREF        = 430.0;
const float RNOMINAL    = 100.0;
const bool  FILTRO_50HZ = false;      // 60 Hz
const uint8_t N_MEDIANA = 3;

const float T_MIN_VALIDA = 0.0;
const float T_MAX_VALIDA = 150.0;

// -- PWM lento --
const uint32_t VENTANA_DEFECTO_MS = 2000;   // §6: ventana de 2 s
const uint32_t VENTANA_MIN_MS     = 500;
const uint32_t VENTANA_MAX_MS     = 10000;

// A 60 Hz cada semiciclo dura 8.33 ms. Un pulso mas corto que esto no lo puede
// reproducir un SSR zero-cross, asi que por debajo del minimo se satura a 0/100.
const uint32_t PULSO_MIN_MS = 20;           // ~2.4 semiciclos

// -- Seguridad --
const uint32_t TIEMPO_MUERTO_MS = 10UL * 60UL * 1000UL;  // 10 min sin comandos
const uint32_t WDT_TIMEOUT_MS   = 8000;

// ---------------------------------------------------------------------------
// ESTADO COMPARTIDO
// Enteros de 32 bits alineados: lectura/escritura atomica en ESP32.
// ---------------------------------------------------------------------------

volatile int  g_dutyComandado   = 0;      // 0-100 %
volatile int  g_salidaArmada    = 0;      // 0 = desarmada
volatile uint32_t g_ventanaMs   = VENTANA_DEFECTO_MS;
volatile uint32_t g_ultimoComando = 0;

volatile float g_dutyReal   = 0.0f;       // medido sobre la ultima ventana
volatile float g_temperatura = NAN;
volatile int   g_sensorOk   = 0;

Adafruit_MAX31865 termo(PIN_CS);

// ---------------------------------------------------------------------------
// SALIDA
// ---------------------------------------------------------------------------

void ponerSSR(bool encendido) {
  digitalWrite(PIN_SSR, encendido ? HIGH : LOW);
}

void desarmar(const char *motivo) {
  g_salidaArmada  = 0;
  g_dutyComandado = 0;
  g_dutyReal      = 0.0f;   // si no, el estado sigue mostrando la ventana previa
  ponerSSR(false);
  Serial.printf("\n*** SALIDA DESARMADA: %s ***\n", motivo);
}

// Tarea de salida: solo conmuta. Nunca llama a nada que bloquee.
void tareaSalida(void *pv) {
  esp_task_wdt_add(NULL);

  const TickType_t periodo = pdMS_TO_TICKS(5);
  TickType_t ultimaEjecucion = xTaskGetTickCount();

  uint32_t inicioVentana = millis();
  uint32_t onAcumulado   = 0;
  uint32_t marcaOn       = 0;
  bool     estadoActual  = false;

  for (;;) {
    esp_task_wdt_reset();

    uint32_t ahora   = millis();
    uint32_t ventana = g_ventanaMs;

    // -- Hombre muerto --
    if (g_dutyComandado > 0 && (ahora - g_ultimoComando) > TIEMPO_MUERTO_MS) {
      desarmar("hombre muerto, 10 min sin comandos");
    }

    // -- Cierre de ventana --
    if ((uint32_t)(ahora - inicioVentana) >= ventana) {
      if (estadoActual) {                     // contabiliza el tramo abierto
        onAcumulado += ahora - marcaOn;
        marcaOn = ahora;
      }
      g_dutyReal = 100.0f * onAcumulado / (float)ventana;
      onAcumulado = 0;
      // Resincroniza en vez de acumular deriva si nos hemos retrasado mucho
      if ((uint32_t)(ahora - inicioVentana) > 2 * ventana) inicioVentana = ahora;
      else                                                inicioVentana += ventana;
    }

    // -- Decision de encendido --
    bool deseado = false;
    if (g_salidaArmada && g_dutyComandado > 0) {
      uint32_t transcurrido = ahora - inicioVentana;
      uint32_t onTime = (uint32_t)((uint64_t)g_dutyComandado * ventana / 100);

      // Saturacion: pulsos que el SSR zero-cross no puede reproducir
      if (onTime < PULSO_MIN_MS)                 onTime = 0;
      else if (onTime > ventana - PULSO_MIN_MS)  onTime = ventana;

      deseado = (transcurrido < onTime);
    }

    // -- Conmutacion y contabilidad --
    if (deseado != estadoActual) {
      if (deseado) {
        marcaOn = ahora;
      } else {
        onAcumulado += ahora - marcaOn;
      }
      ponerSSR(deseado);
      estadoActual = deseado;
    }

    vTaskDelayUntil(&ultimaEjecucion, periodo);
  }
}

// ---------------------------------------------------------------------------
// SENSOR
// ---------------------------------------------------------------------------

inline float rawAGrados(uint16_t raw) {
  return termo.calculateTemperature(raw, RNOMINAL, RREF);
}

int comparaU16(const void *a, const void *b) {
  int d = (int)*(const uint16_t *)a - (int)*(const uint16_t *)b;
  return (d > 0) - (d < 0);
}

uint16_t leerRawMediana() {
  uint16_t m[N_MEDIANA];
  for (uint8_t i = 0; i < N_MEDIANA; i++) m[i] = termo.readRTD();
  qsort(m, N_MEDIANA, sizeof(uint16_t), comparaU16);
  return m[N_MEDIANA / 2];
}

// Tarea de sensor: puede bloquear, no afecta al PWM.
void tareaSensor(void *pv) {
  const TickType_t periodo = pdMS_TO_TICKS(1000);
  TickType_t ultimaEjecucion = xTaskGetTickCount();

  for (;;) {
    float t = rawAGrados(leerRawMediana());
    uint8_t falla = termo.readFault();
    if (falla) termo.clearFault();

    bool ok = !falla && !isnan(t) && t >= T_MIN_VALIDA && t <= T_MAX_VALIDA;

    g_temperatura = t;
    g_sensorOk    = ok ? 1 : 0;

    // §7.2: falla de sensor -> salida a 0 y desarme
    if (!ok && g_salidaArmada) {
      if (falla)                 desarmar("flag de falla del MAX31865");
      else if (isnan(t))         desarmar("lectura NaN del sensor");
      else                       desarmar("temperatura fuera del rango 0-150 C");
    }

    vTaskDelayUntil(&ultimaEjecucion, periodo);
  }
}

// ---------------------------------------------------------------------------
// CONSOLA
// ---------------------------------------------------------------------------

void mostrarEstado() {
  Serial.println(F("\n--- ESTADO ---"));
  Serial.printf("Salida        : %s\n", g_salidaArmada ? "ARMADA" : "desarmada");
  Serial.printf("Duty comandado: %d %%\n", g_dutyComandado);
  Serial.printf("Duty real     : %.2f %% (ultima ventana)\n", g_dutyReal);
  Serial.printf("Ventana       : %lu ms\n", (unsigned long)g_ventanaMs);
  Serial.printf("Pulso minimo  : %lu ms (satura a 0/100 fuera de eso)\n",
                (unsigned long)PULSO_MIN_MS);
  Serial.printf("Temperatura   : %.3f C  [%s]\n", g_temperatura,
                g_sensorOk ? "OK" : "FALLA");
  Serial.printf("Ultimo comando: hace %lu s\n",
                (unsigned long)((millis() - g_ultimoComando) / 1000));
  Serial.println();
}

void fijarDuty(int d) {
  if (d < 0) d = 0;
  if (d > 100) d = 100;
  if (d > 0 && !g_salidaArmada) {
    Serial.println(F("[!] Salida desarmada. Usa 'armar' primero."));
    return;
  }
  g_dutyComandado = d;
  Serial.printf("Duty -> %d %%\n", d);
}

void cmdSecuencia() {
  if (!g_salidaArmada) { Serial.println(F("[!] Usa 'armar' primero.")); return; }
  const int pasos[] = {0, 25, 50, 75, 100, 0};
  Serial.println(F("\n--- BARRIDO: 3 ventanas por escalon ---"));
  Serial.println(F("comandado  real     error"));
  for (uint8_t i = 0; i < 6; i++) {
    g_dutyComandado = pasos[i];
    g_ultimoComando = millis();
    // Deja pasar 3 ventanas: la primera es transitoria, se mide la ultima
    vTaskDelay(pdMS_TO_TICKS(3 * g_ventanaMs));
    float real = g_dutyReal;
    Serial.printf("  %3d %%    %6.2f %%  %+.2f\n", pasos[i], real, real - pasos[i]);
    if (!g_salidaArmada) { Serial.println(F("Abortado: salida desarmada.")); return; }
  }
  Serial.println(F("Barrido terminado. Duty en 0.\n"));
}

// Mide el ruido de la PT100 con el SSR quieto y con el SSR conmutando.
void cmdEmi() {
  Serial.println(F("\n--- INTERFERENCIA DEL SSR SOBRE LA PT100 ---"));

  struct { const char *etiqueta; int duty; } fases[] = {
    {"SSR en reposo (duty 0)",   0},
    {"SSR conmutando (duty 50)", 50}
  };

  double sd[2] = {0, 0};
  for (uint8_t f = 0; f < 2; f++) {
    if (fases[f].duty > 0 && !g_salidaArmada) {
      Serial.println(F("[!] Fase con carga omitida: salida desarmada."));
      return;
    }
    g_dutyComandado = fases[f].duty;
    g_ultimoComando = millis();
    vTaskDelay(pdMS_TO_TICKS(2000));            // deja asentar

    double suma = 0, sumaSq = 0;
    const uint16_t N = 40;
    for (uint16_t i = 0; i < N; i++) {
      float t = rawAGrados(termo.readRTD());
      suma += t; sumaSq += (double)t * t;
    }
    double media = suma / N;
    sd[f] = sqrt(fmax(0.0, sumaSq / N - media * media));
    Serial.printf("%-26s media %.3f C, sd %.4f C\n", fases[f].etiqueta, media, sd[f]);
  }

  g_dutyComandado = 0;
  Serial.printf("Degradacion: %.1fx\n", sd[1] / fmax(sd[0], 1e-6));
  if (sd[1] < 3 * fmax(sd[0], 1e-4))
    Serial.println(F("[OK] El ruido conducido del SSR no degrada la medicion."));
  else
    Serial.println(F("[X] Ruido apreciable. Separar el cable de la PT100 del de potencia,"));
  Serial.println();
}

void ayuda() {
  Serial.println(F("Comandos: armar | desarmar | d <0-100> | off | on | v <ms> |"));
  Serial.println(F("          estado | secuencia | emi | help"));
}

void procesarComando(String cmd) {
  cmd.trim(); cmd.toLowerCase();
  if (!cmd.length()) return;
  g_ultimoComando = millis();

  if (cmd == "armar") {
    if (!g_sensorOk) { Serial.println(F("[!] Sensor en falla. No se arma.")); return; }
    g_salidaArmada = 1;
    Serial.println(F("*** SALIDA ARMADA. La carga puede energizarse. ***"));
  }
  else if (cmd == "desarmar")  desarmar("comando del operador");
  else if (cmd == "off")       fijarDuty(0);
  else if (cmd == "on")        fijarDuty(100);
  else if (cmd.startsWith("d "))  fijarDuty(cmd.substring(2).toInt());
  else if (cmd.startsWith("v ")) {
    long v = cmd.substring(2).toInt();
    if (v < (long)VENTANA_MIN_MS || v > (long)VENTANA_MAX_MS) {
      Serial.printf("[!] Ventana fuera de %lu-%lu ms\n",
                    (unsigned long)VENTANA_MIN_MS, (unsigned long)VENTANA_MAX_MS);
    } else {
      g_ventanaMs = v;
      Serial.printf("Ventana -> %ld ms\n", v);
    }
  }
  else if (cmd == "estado")    mostrarEstado();
  else if (cmd == "secuencia") cmdSecuencia();
  else if (cmd == "emi")       cmdEmi();
  else if (cmd == "help")      ayuda();
  else { Serial.printf("Comando desconocido: \"%s\"\n", cmd.c_str()); ayuda(); }
}

// ---------------------------------------------------------------------------

void setup() {
  // SEGURIDAD (§7.2): estado seguro antes que nada.
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, LOW);

  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n================================================"));
  Serial.println(F(" PID AUTOCLAVE 75X - Paso 2: PWM lento para SSR"));
  Serial.println(F(" *** SOLO CON LAMPARA, NUNCA CON LA RESISTENCIA ***"));
  Serial.println(F(" Salida DESARMADA. Usa 'armar' para habilitarla."));
  Serial.println(F("================================================"));

  termo.begin(MAX31865_3WIRE);
  termo.enable50Hz(FILTRO_50HZ);
  termo.clearFault();
  delay(100);

  // Watchdog sobre la tarea de salida (§7.2)
  esp_task_wdt_config_t wdt = {};
  wdt.timeout_ms     = WDT_TIMEOUT_MS;
  wdt.idle_core_mask = 0;
  wdt.trigger_panic  = true;
  // El core de Arduino ya inicializa el TWDT, asi que reconfigurar es lo normal;
  // esp_task_wdt_init() aqui solo imprimiria "TWDT already initialized".
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);

  g_ultimoComando = millis();

  xTaskCreatePinnedToCore(tareaSalida, "salida", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(tareaSensor, "sensor", 4096, NULL, 2, NULL, 1);

  Serial.printf("Ventana PWM: %lu ms | pulso minimo: %lu ms\n",
                (unsigned long)g_ventanaMs, (unsigned long)PULSO_MIN_MS);
  ayuda();
}

void loop() {
  if (Serial.available()) {
    procesarComando(Serial.readStringUntil('\n'));
  }
  delay(20);
}
