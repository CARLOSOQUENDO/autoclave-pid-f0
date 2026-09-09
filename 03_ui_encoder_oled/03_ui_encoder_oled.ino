/*
 * 03_ui_encoder_oled.ino
 * PID Autoclave All American 75X - Paso 3 del orden de desarrollo
 *
 * Interfaz de usuario: encoder KY-040 + OLED SSD1306 128x64 en 0x3C.
 * Incorpora las etapas ya validadas de los pasos 1 y 2 (PT100 y PWM lento).
 *
 * *** LA SALIDA SIGUE SIENDO SOLO PARA LAMPARA / SERIE DE PRUEBA ***
 *
 * ARQUITECTURA (seccion 8)
 * ------------------------
 *   tareaSalida  (prio 3, 5 ms)   -> conmuta el SSR, nunca se bloquea
 *   tareaSensor  (prio 2, 1 s)    -> lee la PT100 (bloquea 225 ms)
 *   tareaUI      (prio 1, 10 ms)  -> encoder; redibuja cada 250 ms o al cambiar
 *   loop()       (prio 1)         -> consola serie
 *
 * La tarea de UI nunca toca el SSR ni espera al sensor: lee variables
 * compartidas. Asi el refresco de pantalla no puede desfasar el PWM.
 *
 * ENCODER
 * -------
 * Cuadratura por PCNT (seccion 6): el filtro antirrebote es por hardware, no
 * hay interrupciones que compitan con las tareas ni debounce por software.
 *
 * Comandos serie: estado | armar | desarmar | d <0-100> | enc | guardar | help
 */

#include <Adafruit_MAX31865.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include "driver/pulse_cnt.h"
#include <math.h>

// Poner a 1 si la pantalla es SH1106 en vez de SSD1306. Sintoma tipico de
// equivocarse: la imagen sale desplazada 2 px en horizontal y con basura en el
// borde. No hace dano, solo se ve mal.
#define CONTROLADOR_SH1106 0

// ---------------------------------------------------------------------------
// PINES (seccion 3)
// ---------------------------------------------------------------------------
const uint8_t PIN_SSR     = 25;
const uint8_t PIN_CS      = 5;
const gpio_num_t PIN_ENC_CLK = GPIO_NUM_26;
const gpio_num_t PIN_ENC_DT  = GPIO_NUM_27;
const uint8_t PIN_ENC_SW  = 14;
const uint8_t PIN_BUZZER  = 33;

// ---------------------------------------------------------------------------
// CONFIGURACION
// ---------------------------------------------------------------------------
const float RREF        = 430.0;
const float RNOMINAL    = 100.0;
const bool  FILTRO_50HZ = false;
const uint8_t N_MEDIANA = 3;

const float T_MIN_VALIDA = 0.0;
const float T_MAX_VALIDA = 150.0;

const uint32_t PULSO_MIN_MS     = 20;
const uint32_t TIEMPO_MUERTO_MS = 10UL * 60UL * 1000UL;
const uint32_t WDT_TIMEOUT_MS   = 8000;

// Pulsos de cuadratura por detente del KY-040. Usar el comando 'enc' para
// comprobarlo: un click deberia dar exactamente 1.
const int PASOS_POR_DETENTE = 4;

const uint32_t PULSACION_LARGA_MS = 600;

// -- Bus I2C de la pantalla --
// 400 kHz permite refrescar el framebuffer completo (1 KB) en ~23 ms. Si la
// alimentacion es ruidosa o el cableado largo, bajar a 100000: es mas inmune,
// pero el refresco pasa a ~92 ms, asi que hay que subir tambien PERIODO_DIBUJO.
const uint32_t VELOCIDAD_I2C   = 400000;
const uint8_t  DIRECCIONES_OLED[] = { 0x3C, 0x3D };

// La UI sondea el encoder rapido y redibuja despacio: redibujar los 1024 bytes
// del framebuffer 25 veces por segundo es desperdicio, y a 100 kHz ni cabria.
const uint32_t PERIODO_SONDEO_MS = 10;
const uint32_t PERIODO_DIBUJO_MS = 250;

// ---------------------------------------------------------------------------
// ESTADO COMPARTIDO
// ---------------------------------------------------------------------------
volatile int      g_dutyComandado = 0;
volatile int      g_salidaArmada  = 0;
volatile uint32_t g_ventanaMs     = 2000;
volatile uint32_t g_ultimoComando = 0;
volatile float    g_dutyReal      = 0.0f;
volatile float    g_temperatura   = NAN;

// Tres estados, no dos: "sin medida todavia" no es "falla". Distinguirlo
// importa en un panel de operador (detectado en el paso 2).
enum EstadoSensor : int { SENSOR_SIN_MEDIDA = 0, SENSOR_OK = 1, SENSOR_FALLA = 2 };
volatile int g_estadoSensor = SENSOR_SIN_MEDIDA;
char g_motivoFalla[40] = "";

// Parametros persistentes (seccion 6: guardar en NVS)
struct Parametros {
  float setpoint;    // C
  float f0Objetivo;  // min
  float kp, ki, kd;
  uint32_t ventanaMs;
} g_par;

Preferences prefs;
Adafruit_MAX31865 termo(PIN_CS);

#if CONTROLADOR_SH1106
  U8G2_SH1106_128X64_NONAME_F_HW_I2C pantalla(U8G2_R0, U8X8_PIN_NONE);
#else
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C pantalla(U8G2_R0, U8X8_PIN_NONE);
#endif

pcnt_unit_handle_t pcntUnidad = NULL;

volatile bool g_pantallaOk  = false;
uint8_t       g_dirPantalla = 0;

// ---------------------------------------------------------------------------
// PARAMETROS / NVS
// ---------------------------------------------------------------------------
void cargarParametros() {
  prefs.begin("autoclave", false);
  g_par.setpoint   = prefs.getFloat("sp",   121.0f);
  g_par.f0Objetivo = prefs.getFloat("f0",    20.0f);
  g_par.kp         = prefs.getFloat("kp",     2.0f);
  g_par.ki         = prefs.getFloat("ki",     0.05f);
  g_par.kd         = prefs.getFloat("kd",     0.0f);
  g_par.ventanaMs  = prefs.getUInt ("vent", 2000);
  g_ventanaMs = g_par.ventanaMs;
}

void guardarParametros() {
  prefs.putFloat("sp",   g_par.setpoint);
  prefs.putFloat("f0",   g_par.f0Objetivo);
  prefs.putFloat("kp",   g_par.kp);
  prefs.putFloat("ki",   g_par.ki);
  prefs.putFloat("kd",   g_par.kd);
  prefs.putUInt ("vent", g_par.ventanaMs);
  Serial.println(F("Parametros guardados en NVS."));
}

// ---------------------------------------------------------------------------
// BUZZER
// ---------------------------------------------------------------------------
void pitido(uint16_t freq, uint16_t ms) { tone(PIN_BUZZER, freq, ms); }
void pitidoClic()  { pitido(2000,  15); }
void pitidoOk()    { pitido(1500,  60); }
void pitidoError() { pitido( 400, 250); }

// Melodia de arranque: arpegio ascendente que remata en una nota aguda
// sostenida. Las notas altas caen cerca de la resonancia tipica de los
// transductores piezoelectricos (2-4 kHz), asi que suenan bastante mas fuerte
// que un pitido grave con la misma tension de excitacion.
void melodiaArranque() {
  struct Nota { uint16_t freq, ms; };
  static const Nota MELODIA[] = {
    {  784,  70 },   // G5
    { 1047,  70 },   // C6
    { 1319,  70 },   // E6
    { 1568,  70 },   // G6
    { 2093, 320 },   // C7, sostenida
  };
  for (uint8_t i = 0; i < sizeof(MELODIA) / sizeof(Nota); i++) {
    tone(PIN_BUZZER, MELODIA[i].freq, MELODIA[i].ms);
    delay(MELODIA[i].ms + 25);
  }
  noTone(PIN_BUZZER);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
}


// ---------------------------------------------------------------------------
// SALIDA (identica al paso 2)
// ---------------------------------------------------------------------------
void ponerSSR(bool on) { digitalWrite(PIN_SSR, on ? HIGH : LOW); }

void desarmar(const char *motivo) {
  g_salidaArmada  = 0;
  g_dutyComandado = 0;
  g_dutyReal      = 0.0f;
  ponerSSR(false);
  Serial.printf("\n*** SALIDA DESARMADA: %s ***\n", motivo);
}

void tareaSalida(void *pv) {
  esp_task_wdt_add(NULL);
  const TickType_t periodo = pdMS_TO_TICKS(5);
  TickType_t ultima = xTaskGetTickCount();

  uint32_t inicioVentana = millis();
  uint32_t onAcumulado = 0, marcaOn = 0;
  bool estadoActual = false;

  for (;;) {
    esp_task_wdt_reset();
    uint32_t ahora = millis();
    uint32_t ventana = g_ventanaMs;

    if (g_dutyComandado > 0 && (ahora - g_ultimoComando) > TIEMPO_MUERTO_MS)
      desarmar("hombre muerto, 10 min sin comandos");

    if ((uint32_t)(ahora - inicioVentana) >= ventana) {
      if (estadoActual) { onAcumulado += ahora - marcaOn; marcaOn = ahora; }
      g_dutyReal = 100.0f * onAcumulado / (float)ventana;
      onAcumulado = 0;
      if ((uint32_t)(ahora - inicioVentana) > 2 * ventana) inicioVentana = ahora;
      else                                                inicioVentana += ventana;
    }

    bool deseado = false;
    if (g_salidaArmada && g_dutyComandado > 0) {
      uint32_t transcurrido = ahora - inicioVentana;
      uint32_t onTime = (uint32_t)((uint64_t)g_dutyComandado * ventana / 100);
      if (onTime < PULSO_MIN_MS)                onTime = 0;
      else if (onTime > ventana - PULSO_MIN_MS) onTime = ventana;
      deseado = (transcurrido < onTime);
    }

    if (deseado != estadoActual) {
      if (deseado) marcaOn = ahora;
      else         onAcumulado += ahora - marcaOn;
      ponerSSR(deseado);
      estadoActual = deseado;
    }
    vTaskDelayUntil(&ultima, periodo);
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

void tareaSensor(void *pv) {
  const TickType_t periodo = pdMS_TO_TICKS(1000);
  TickType_t ultima = xTaskGetTickCount();

  for (;;) {
    float t = rawAGrados(leerRawMediana());
    uint8_t falla = termo.readFault();
    if (falla) termo.clearFault();

    const char *motivo = NULL;
    if (falla)                              motivo = "flag MAX31865";
    else if (isnan(t))                      motivo = "lectura NaN";
    else if (t < T_MIN_VALIDA)              motivo = "T bajo rango";
    else if (t > T_MAX_VALIDA)              motivo = "T sobre rango";

    g_temperatura  = t;
    g_estadoSensor = motivo ? SENSOR_FALLA : SENSOR_OK;

    if (motivo) {
      strncpy(g_motivoFalla, motivo, sizeof(g_motivoFalla) - 1);
      if (g_salidaArmada) desarmar(motivo);     // §7.2
    } else {
      g_motivoFalla[0] = '\0';
    }
    vTaskDelayUntil(&ultima, periodo);
  }
}

// ---------------------------------------------------------------------------
// ENCODER (PCNT)
// ---------------------------------------------------------------------------
void iniciarEncoder() {
  pcnt_unit_config_t cfgU = {};
  cfgU.low_limit  = -1000;
  cfgU.high_limit =  1000;
  ESP_ERROR_CHECK(pcnt_new_unit(&cfgU, &pcntUnidad));

  // Filtro antirrebote por hardware: descarta pulsos de menos de 1 us
  pcnt_glitch_filter_config_t cfgF = {};
  cfgF.max_glitch_ns = 1000;
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcntUnidad, &cfgF));

  pcnt_chan_config_t cfgA = {};
  cfgA.edge_gpio_num  = PIN_ENC_CLK;
  cfgA.level_gpio_num = PIN_ENC_DT;
  pcnt_channel_handle_t canA = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcntUnidad, &cfgA, &canA));

  pcnt_chan_config_t cfgB = {};
  cfgB.edge_gpio_num  = PIN_ENC_DT;
  cfgB.level_gpio_num = PIN_ENC_CLK;
  pcnt_channel_handle_t canB = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcntUnidad, &cfgB, &canB));

  // Decodificacion en cuadratura x4
  pcnt_channel_set_edge_action(canA, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(canA, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_channel_set_edge_action(canB, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(canB, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  ESP_ERROR_CHECK(pcnt_unit_enable(pcntUnidad));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcntUnidad));
  ESP_ERROR_CHECK(pcnt_unit_start(pcntUnidad));

  // El KY-040 suele traer pull-ups de 10k, pero no todos los modulos los
  // pueblan. Los internos no estorban si ya existen los externos.
  gpio_set_pull_mode(PIN_ENC_CLK, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(PIN_ENC_DT,  GPIO_PULLUP_ONLY);
}

// Devuelve detentes girados desde la ultima llamada, conservando el resto.
int leerGiro() {
  static int resto = 0;
  int cuenta = 0;
  pcnt_unit_get_count(pcntUnidad, &cuenta);
  pcnt_unit_clear_count(pcntUnidad);
  cuenta += resto;
  int detentes = cuenta / PASOS_POR_DETENTE;
  resto = cuenta - detentes * PASOS_POR_DETENTE;
  return detentes;
}

// Pulsador: 0 = nada, 1 = pulsacion corta, 2 = pulsacion larga
int leerPulsador() {
  static bool     estabaPulsado = false;
  static uint32_t tPulsacion = 0;
  static bool     largaEmitida = false;

  bool pulsado = (digitalRead(PIN_ENC_SW) == LOW);
  uint32_t ahora = millis();

  if (pulsado && !estabaPulsado) {
    estabaPulsado = true; tPulsacion = ahora; largaEmitida = false;
  } else if (pulsado && estabaPulsado && !largaEmitida &&
             (ahora - tPulsacion) >= PULSACION_LARGA_MS) {
    largaEmitida = true;
    return 2;
  } else if (!pulsado && estabaPulsado) {
    estabaPulsado = false;
    if (!largaEmitida) return 1;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// INTERFAZ
//
// Dos niveles, porque hay dos usuarios distintos:
//   - Menu principal: lo maneja el microbiologo. Lenguaje llano, sin jerga de
//     control. Temperatura, letalidad, y poco mas.
//   - Configuracion: ajustes tecnicos (PID, ventana de PWM, modo manual).
//     Quien no sabe que es Kp no tiene por que tropezarse con ello.
//
// Cada item lleva una linea de ayuda que se muestra abajo al seleccionarlo:
// mas util que un manual que nadie va a tener al lado del equipo.
// ---------------------------------------------------------------------------
const char *VERSION_FW = "0.3.0";
const char *AUTOR_FW   = "BY_Oquendo";

enum Vista { VISTA_PRINCIPAL, VISTA_MENU, VISTA_EDITAR, VISTA_MANUAL, VISTA_INFO,
             VISTA_CONFIRMAR, VISTA_CHECKLIST, VISTA_LISTO };
Vista g_vista = VISTA_PRINCIPAL;

enum TipoItem : uint8_t { IT_VALOR, IT_SUBMENU, IT_ACCION };
enum Accion   : uint8_t { AC_MANUAL, AC_GUARDAR, AC_BUZZER, AC_VOLVER, AC_INFO, AC_INICIAR };

struct ItemMenu {
  const char *etiqueta;
  const char *ayuda;        // max 25 caracteres: es lo que cabe en la fuente 5x8
  uint8_t     tipo;
  float      *valor;
  float       minimo, maximo, paso;
  uint8_t     decimales;
  const char *unidad;
  uint8_t     arg;          // submenu destino, o codigo de accion
};

float g_ventanaEditable = 2000;

const ItemMenu MENU_OPERADOR[] = {
  { "Iniciar ciclo",  "Arranca la esterilizacion", IT_ACCION, NULL, 0, 0, 0, 0, NULL, AC_INICIAR },
  { "Temperatura",   "Consigna de la meseta",    IT_VALOR,   &g_par.setpoint,    50,  135, 0.5f, 1, "C",   0 },
  { "Letalidad F0",  "Min. equivalentes a 121C", IT_VALOR,   &g_par.f0Objetivo,   1,  120, 1.0f, 0, "min", 0 },
  { "Configuracion", "Ajustes tecnicos",         IT_SUBMENU, NULL, 0, 0, 0, 0, NULL, 1 },
  { "Informacion",   "Version del equipo",       IT_ACCION,  NULL, 0, 0, 0, 0, NULL, AC_INFO },
  { "Volver",        "Salir al panel principal", IT_ACCION,  NULL, 0, 0, 0, 0, NULL, AC_VOLVER },
};

const ItemMenu MENU_TECNICO[] = {
  { "Modo manual",   "Salida manual de pruebas", IT_ACCION, NULL, 0, 0, 0, 0, NULL, AC_MANUAL },
  { "Control Kp",    "Ganancia proporcional",    IT_VALOR,  &g_par.kp,   0,   100, 0.1f,  2, NULL, 0 },
  { "Control Ki",    "Ganancia integral",        IT_VALOR,  &g_par.ki,   0,    10, 0.01f, 3, NULL, 0 },
  { "Control Kd",    "Ganancia derivativa",      IT_VALOR,  &g_par.kd,   0,   100, 0.1f,  2, NULL, 0 },
  { "Ventana PWM",   "Periodo de ciclo del SSR", IT_VALOR,  &g_ventanaEditable, 500, 10000, 100.0f, 0, "ms", 0 },
  { "Probar sonido", "Comprueba el zumbador",    IT_ACCION, NULL, 0, 0, 0, 0, NULL, AC_BUZZER },
  { "Guardar",       "Graba en memoria",         IT_ACCION, NULL, 0, 0, 0, 0, NULL, AC_GUARDAR },
  { "Volver",        "Vuelve al menu anterior",  IT_ACCION, NULL, 0, 0, 0, 0, NULL, AC_VOLVER },
};

struct Menu { const char *titulo; const ItemMenu *items; uint8_t n; int8_t padre; };

const Menu MENUS[] = {
  { "MENU",          MENU_OPERADOR, sizeof(MENU_OPERADOR) / sizeof(ItemMenu), -1 },
  { "CONFIGURACION", MENU_TECNICO,  sizeof(MENU_TECNICO)  / sizeof(ItemMenu),  0 },
};
const uint8_t N_MENUS = sizeof(MENUS) / sizeof(Menu);

uint8_t g_menuActual = 0;
int     g_cursor[N_MENUS] = {0};   // se recuerda por menu, para volver donde estabas
int     g_scroll[N_MENUS] = {0};
const ItemMenu *g_itemEditando = NULL;

// --- Arranque robusto de la OLED ------------------------------------------
// Sondea una direccion del bus I2C
bool respondeI2C(uint8_t dir) {
  Wire.beginTransmission(dir);
  return Wire.endTransmission() == 0;
}

// Con alimentacion externa (fuente AC-DC) el rail puede subir mas despacio que
// con USB, y el SSD1306 aun no ha terminado su reset interno cuando el firmware
// intenta inicializarlo: la pantalla se queda negra aunque todo este bien
// cableado. Por eso se sondea el bus y se reintenta en vez de dar por hecho que
// esta lista. Tambien detecta la direccion (0x3C o 0x3D) en vez de asumirla.
bool iniciarPantalla(uint8_t intentos) {
  Wire.begin(21, 22);
  Wire.setClock(VELOCIDAD_I2C);

  for (uint8_t i = 1; i <= intentos; i++) {
    for (uint8_t k = 0; k < sizeof(DIRECCIONES_OLED); k++) {
      uint8_t dir = DIRECCIONES_OLED[k];
      if (respondeI2C(dir)) {
        g_dirPantalla = dir;
        pantalla.setI2CAddress(dir << 1);      // u8g2 usa la direccion de 8 bits
        pantalla.begin();
        pantalla.setBusClock(VELOCIDAD_I2C);
        pantalla.setContrast(200);
        g_pantallaOk = true;
        Serial.printf("OLED detectada en 0x%02X (intento %u).\n", dir, i);
        return true;
      }
    }
    Serial.printf("OLED no responde en 0x3C ni 0x3D (intento %u/%u)...\n", i, intentos);
    delay(250);
  }

  g_pantallaOk = false;
  Serial.println(F("[X] OLED no detectada. Revisar alimentacion (3V3 >= 3.2 V),"));
  Serial.println(F("    continuidad de SDA=21 / SCL=22 y GND comun."));
  return false;
}

// --- Pantalla principal (sin cambios: es la que ya funciona bien) ----------
void dibujarPrincipal() {
  char buf[24];

  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, g_salidaArmada ? "ARMADA" : "desarmada");

  const char *etiqueta;
  switch (g_estadoSensor) {
    case SENSOR_OK:    etiqueta = "OK";    break;
    case SENSOR_FALLA: etiqueta = "FALLA"; break;
    default:           etiqueta = "---";   break;
  }
  pantalla.drawStr(128 - pantalla.getStrWidth(etiqueta), 8, etiqueta);
  pantalla.drawHLine(0, 11, 128);

  float t = g_temperatura;
  if (g_estadoSensor == SENSOR_OK) snprintf(buf, sizeof(buf), "%.1f", t);
  else                             snprintf(buf, sizeof(buf), "--.-");
  pantalla.setFont(u8g2_font_logisoso20_tn);
  int ancho = pantalla.getStrWidth(buf);
  pantalla.drawStr(64 - (ancho + 14) / 2, 38, buf);
  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(64 + (ancho + 14) / 2 - 12, 30, "C");

  snprintf(buf, sizeof(buf), "SP %.1f", g_par.setpoint);
  pantalla.drawStr(0, 50, buf);
  snprintf(buf, sizeof(buf), "%3d%%", g_dutyComandado);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf), 50, buf);

  pantalla.drawFrame(0, 54, 128, 9);
  int ancho_barra = (g_dutyComandado * 126) / 100;
  if (ancho_barra > 0) pantalla.drawBox(1, 55, ancho_barra, 7);

  if (g_estadoSensor == SENSOR_FALLA && g_motivoFalla[0]) {
    pantalla.setDrawColor(0); pantalla.drawBox(0, 40, 128, 11);
    pantalla.setDrawColor(1); pantalla.drawStr(0, 49, g_motivoFalla);
  }
}

// --- Menu: 3 items en fuente grande + linea de ayuda -----------------------
void dibujarMenu() {
  const Menu &m = MENUS[g_menuActual];
  int &cur = g_cursor[g_menuActual];
  int &scr = g_scroll[g_menuActual];
  const uint8_t VISIBLES = 3;
  char buf[32];

  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, m.titulo);
  snprintf(buf, sizeof(buf), "%d/%d", cur + 1, m.n);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf), 8, buf);
  pantalla.drawHLine(0, 11, 128);

  if (cur < scr)             scr = cur;
  if (cur >= scr + VISIBLES) scr = cur - VISIBLES + 1;

  pantalla.setFont(u8g2_font_7x13_tf);
  for (uint8_t i = 0; i < VISIBLES; i++) {
    int idx = scr + i;
    if (idx >= m.n) break;
    int y = 24 + i * 13;
    const ItemMenu &it = m.items[idx];

    if (idx == cur) {
      pantalla.drawBox(0, y - 11, 128, 13);
      pantalla.setDrawColor(0);
    }
    pantalla.drawStr(3, y, it.etiqueta);

    if (it.tipo == IT_VALOR) {
      if (it.unidad) snprintf(buf, sizeof(buf), "%.*f%s", it.decimales, *it.valor, it.unidad);
      else           snprintf(buf, sizeof(buf), "%.*f",   it.decimales, *it.valor);
      pantalla.drawStr(125 - pantalla.getStrWidth(buf), y, buf);
    } else if (it.tipo == IT_SUBMENU) {
      pantalla.drawStr(117, y, ">");
    }
    pantalla.setDrawColor(1);
  }

  pantalla.drawHLine(0, 53, 128);
  pantalla.setFont(u8g2_font_5x8_tf);
  pantalla.drawStr(1, 62, m.items[cur].ayuda);
}

void dibujarEditar() {
  const ItemMenu &it = *g_itemEditando;
  char buf[32];

  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, it.etiqueta);
  pantalla.drawHLine(0, 11, 128);

  snprintf(buf, sizeof(buf), "%.*f", it.decimales, *it.valor);
  pantalla.setFont(u8g2_font_logisoso20_tn);
  int w = pantalla.getStrWidth(buf);
  int anchoUnidad = 0;
  if (it.unidad) {
    pantalla.setFont(u8g2_font_7x13_tf);
    anchoUnidad = pantalla.getStrWidth(it.unidad) + 4;
  }
  int x = 64 - (w + anchoUnidad) / 2;
  pantalla.setFont(u8g2_font_logisoso20_tn);
  pantalla.drawStr(x, 40, buf);
  if (it.unidad) {
    pantalla.setFont(u8g2_font_7x13_tf);
    pantalla.drawStr(x + w + 4, 40, it.unidad);
  }

  pantalla.setFont(u8g2_font_5x8_tf);
  snprintf(buf, sizeof(buf), "min %.*f   max %.*f",
           it.decimales, it.minimo, it.decimales, it.maximo);
  pantalla.drawStr(64 - pantalla.getStrWidth(buf) / 2, 52, buf);
  const char *pista = "girar cambia | clic OK";
  pantalla.drawStr(64 - pantalla.getStrWidth(pista) / 2, 62, pista);
}

void dibujarInfo() {
  char buf[32];

  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, "INFORMACION");
  pantalla.drawHLine(0, 11, 128);

  pantalla.setFont(u8g2_font_7x13_tf);
  pantalla.drawStr(2, 25, "PID AUTOCLAVE");

  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(2, 36, "All American 75X");
  snprintf(buf, sizeof(buf), "Firmware %s", VERSION_FW);
  pantalla.drawStr(2, 46, buf);

  pantalla.drawHLine(0, 50, 128);
  pantalla.setFont(u8g2_font_7x13_tf);
  pantalla.drawStr(128 - pantalla.getStrWidth(AUTOR_FW) - 3, 62, AUTOR_FW);
}

void dibujarManual() {
  char buf[32];
  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, "MODO MANUAL");
  pantalla.drawHLine(0, 11, 128);

  if (!g_salidaArmada) {
    pantalla.setFont(u8g2_font_7x13_tf);
    pantalla.drawStr(0, 27, "Salida");
    pantalla.drawStr(0, 41, "DESARMADA");
    pantalla.setFont(u8g2_font_5x8_tf);
    pantalla.drawStr(0, 52, "Consola: armar");
    pantalla.drawStr(0, 62, "pulsacion larga = salir");
    return;
  }

  snprintf(buf, sizeof(buf), "%d", g_dutyComandado);
  pantalla.setFont(u8g2_font_logisoso20_tn);
  pantalla.drawStr(50 - pantalla.getStrWidth(buf) / 2, 38, buf);
  pantalla.setFont(u8g2_font_7x13_tf);
  pantalla.drawStr(74, 38, "% salida");

  pantalla.drawFrame(0, 44, 128, 9);
  int w = (g_dutyComandado * 126) / 100;
  if (w > 0) pantalla.drawBox(1, 45, w, 7);

  pantalla.setFont(u8g2_font_5x8_tf);
  snprintf(buf, sizeof(buf), "real %.1f%%   larga = salir", g_dutyReal);
  pantalla.drawStr(0, 62, buf);
}

// --- Dialogo de confirmacion y checklist previo ----------------------------
//
// El mismo widget Si/No sirve para dos cosas: confirmar el arranque del ciclo
// (evita falsos inicios al navegar) y recorrer el checklist previo de la
// Fase 0 (§5), que es obligatorio antes de energizar el SSR.
//
// El cursor arranca SIEMPRE en NO. En un recipiente a presion el valor por
// defecto de cualquier pregunta tiene que ser el que no hace nada.

int     g_siNo = 0;                 // 0 = NO (defecto seguro), 1 = SI
uint8_t g_puntoChecklist = 0;

struct PuntoChecklist { const char *l1; const char *l2; };

// Maximo 21 caracteres por linea con la fuente 6x10.
const PuntoChecklist CHECKLIST[] = {
  { "Nivel de agua OK y",   "resistencia sumergida" },
  { "Valvula de seguridad", "libre y sin obstruir"  },
  { "Tapa asegurada y",     "correctamente sellada" },
  { "Carga en capas de",    "5 cm o menos"          },
};
const uint8_t N_CHECKLIST = sizeof(CHECKLIST) / sizeof(PuntoChecklist);

// Selector NO / SI. NO a la izquierda porque es el destino del gesto de
// retroceso y el que queremos que sea mas facil de elegir sin querer.
void dibujarSiNo(int y) {
  pantalla.setFont(u8g2_font_7x13_tf);
  const int xNo = 30, xSi = 86;

  if (g_siNo == 0) { pantalla.drawBox(xNo - 7, y - 11, 29, 14); pantalla.setDrawColor(0); }
  pantalla.drawStr(xNo, y, "NO");
  pantalla.setDrawColor(1);

  if (g_siNo == 1) { pantalla.drawBox(xSi - 7, y - 11, 29, 14); pantalla.setDrawColor(0); }
  pantalla.drawStr(xSi, y, "SI");
  pantalla.setDrawColor(1);
}

void dibujarConfirmar() {
  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, "CONFIRMAR");
  pantalla.drawHLine(0, 11, 128);

  pantalla.drawStr(0, 25, "Iniciar ciclo de");
  pantalla.drawStr(0, 36, "esterilizacion?");

  dibujarSiNo(51);

  pantalla.setFont(u8g2_font_5x8_tf);
  const char *pista = "girar elige | clic acepta";
  pantalla.drawStr(64 - pantalla.getStrWidth(pista) / 2, 63, pista);
}

void dibujarChecklist() {
  char buf[16];
  const PuntoChecklist &p = CHECKLIST[g_puntoChecklist];

  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, "VERIFICACION");
  snprintf(buf, sizeof(buf), "%u/%u", g_puntoChecklist + 1, N_CHECKLIST);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf), 8, buf);
  pantalla.drawHLine(0, 11, 128);

  pantalla.drawStr(0, 25, p.l1);
  if (p.l2[0]) pantalla.drawStr(0, 36, p.l2);

  dibujarSiNo(51);

  pantalla.setFont(u8g2_font_5x8_tf);
  const char *pista = "NO cancela la verificacion";
  pantalla.drawStr(64 - pantalla.getStrWidth(pista) / 2, 63, pista);
}

void dibujarListo() {
  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, "VERIFICACION");
  pantalla.drawHLine(0, 11, 128);

  pantalla.setFont(u8g2_font_7x13_tf);
  pantalla.drawStr(2, 27, "COMPLETA");

  pantalla.setFont(u8g2_font_5x8_tf);
  pantalla.drawStr(0, 40, "Los 4 puntos confirmados.");
  pantalla.drawStr(0, 49, "La secuencia del ciclo");
  pantalla.drawStr(0, 58, "llega en el paso 6.");
}

void refrescarPantalla() {
  if (!g_pantallaOk) return;
  pantalla.clearBuffer();
  switch (g_vista) {
    case VISTA_PRINCIPAL: dibujarPrincipal(); break;
    case VISTA_MENU:      dibujarMenu();      break;
    case VISTA_EDITAR:    dibujarEditar();    break;
    case VISTA_MANUAL:    dibujarManual();    break;
    case VISTA_INFO:      dibujarInfo();      break;
    case VISTA_CONFIRMAR: dibujarConfirmar(); break;
    case VISTA_CHECKLIST: dibujarChecklist(); break;
    case VISTA_LISTO:     dibujarListo();     break;
  }
  pantalla.sendBuffer();
}

// --- Navegacion ------------------------------------------------------------
void confirmarEdicion() {
  if (g_itemEditando && g_itemEditando->valor == &g_ventanaEditable) {
    g_par.ventanaMs = (uint32_t)g_ventanaEditable;
    g_ventanaMs     = g_par.ventanaMs;
  }
}

// Aborta la secuencia de arranque y vuelve al panel principal. La salida no se
// toca porque en esta fase todavia no se ha energizado nada (§5 Fase 0).
void cancelarArranque(const char *motivo) {
  g_puntoChecklist = 0;
  g_siNo  = 0;
  g_vista = VISTA_PRINCIPAL;
  Serial.printf("# Arranque de ciclo cancelado: %s\n", motivo);
}

void subirNivel() {
  if (MENUS[g_menuActual].padre >= 0) g_menuActual = MENUS[g_menuActual].padre;
  else                                g_vista = VISTA_PRINCIPAL;
}

void aplicarGiro(int giro) {
  switch (g_vista) {
    case VISTA_PRINCIPAL:
    case VISTA_INFO:
    case VISTA_LISTO:
      break;

    case VISTA_CONFIRMAR:
    case VISTA_CHECKLIST: {
      // Horario elige SI, antihorario NO. Mas predecible que alternar.
      int nuevo = (giro > 0) ? 1 : 0;
      if (nuevo != g_siNo) { g_siNo = nuevo; pitidoClic(); }
      break;
    }

    case VISTA_MENU: {
      const Menu &m = MENUS[g_menuActual];
      int &cur = g_cursor[g_menuActual];
      cur += giro;
      if (cur < 0)    cur = 0;
      if (cur >= m.n) cur = m.n - 1;
      pitidoClic();
      break;
    }

    case VISTA_EDITAR: {
      const ItemMenu &it = *g_itemEditando;
      float v = *it.valor + giro * it.paso;
      if (v < it.minimo) v = it.minimo;
      if (v > it.maximo) v = it.maximo;
      *it.valor = v;
      pitidoClic();
      break;
    }

    case VISTA_MANUAL: {
      if (!g_salidaArmada) break;
      int d = g_dutyComandado + giro;
      if (d < 0)   d = 0;
      if (d > 100) d = 100;
      g_dutyComandado = d;
      g_ultimoComando = millis();
      pitidoClic();
      break;
    }
  }
}

void aplicarPulsacion(int tipo) {
  if (tipo == 2) {                                  // larga = atras
    switch (g_vista) {
      case VISTA_MANUAL: g_dutyComandado = 0; g_vista = VISTA_MENU; break;
      case VISTA_EDITAR: confirmarEdicion();  g_vista = VISTA_MENU; break;
      case VISTA_INFO:   g_vista = VISTA_MENU; break;
      case VISTA_MENU:   subirNivel(); break;
      case VISTA_CONFIRMAR:
      case VISTA_CHECKLIST:
      case VISTA_LISTO:  cancelarArranque("cancelado por el operador"); break;
      default:           g_vista = VISTA_PRINCIPAL; break;
    }
    pitidoOk();
    return;
  }
  if (tipo != 1) return;

  switch (g_vista) {
    case VISTA_PRINCIPAL:
      g_menuActual = 0; g_vista = VISTA_MENU; pitidoOk();
      break;

    case VISTA_INFO:
      g_vista = VISTA_MENU; pitidoOk();
      break;

    case VISTA_EDITAR:
      confirmarEdicion(); g_vista = VISTA_MENU; pitidoOk();
      break;

    case VISTA_MANUAL:
      break;                                        // solo sale con larga

    case VISTA_CONFIRMAR:
      if (g_siNo == 1) {
        g_puntoChecklist = 0;
        g_siNo  = 0;                                // cada punto vuelve a NO
        g_vista = VISTA_CHECKLIST;
        pitidoOk();
      } else {
        g_vista = VISTA_MENU;
        pitidoOk();
      }
      break;

    case VISTA_CHECKLIST:
      if (g_siNo != 1) {                            // §5 Fase 0: un NO aborta
        cancelarArranque("punto del checklist no confirmado");
        break;
      }
      g_puntoChecklist++;
      g_siNo = 0;
      if (g_puntoChecklist >= N_CHECKLIST) {
        g_vista = VISTA_LISTO;
        pitidoOk(); delay(120); pitidoOk();
      } else {
        pitidoOk();
      }
      break;

    case VISTA_LISTO:
      g_vista = VISTA_PRINCIPAL;
      pitidoOk();
      break;

    case VISTA_MENU: {
      const ItemMenu &it = MENUS[g_menuActual].items[g_cursor[g_menuActual]];

      if (it.tipo == IT_VALOR) {
        if (it.valor == &g_ventanaEditable) g_ventanaEditable = g_ventanaMs;
        g_itemEditando = &it;
        g_vista = VISTA_EDITAR;
      } else if (it.tipo == IT_SUBMENU) {
        g_menuActual = it.arg;
      } else {
        switch (it.arg) {
          case AC_MANUAL:  g_vista = VISTA_MANUAL; break;
          case AC_INFO:    g_vista = VISTA_INFO;   break;
          case AC_GUARDAR: g_par.ventanaMs = (uint32_t)g_ventanaEditable;
                           guardarParametros(); break;
          case AC_BUZZER:  pitidoOk(); delay(300); pitidoError(); break;
          case AC_VOLVER:  subirNivel(); break;
          case AC_INICIAR: g_siNo = 0;                 // arranca siempre en NO
                           g_vista = VISTA_CONFIRMAR; break;
        }
      }
      pitidoOk();
      break;
    }
  }
}

void tareaUI(void *pv) {
  const TickType_t periodo = pdMS_TO_TICKS(PERIODO_SONDEO_MS);
  TickType_t ultima = xTaskGetTickCount();
  uint32_t proximoDibujo = 0, proximoReintento = 0;
  bool sucia = true;

  for (;;) {
    // El encoder se sondea rapido para que el giro se sienta fluido...
    int giro = leerGiro();
    if (giro) { aplicarGiro(giro); sucia = true; }

    int pulso = leerPulsador();
    if (pulso) { aplicarPulsacion(pulso); sucia = true; }

    uint32_t ahora = millis();

    // ...pero el framebuffer solo se envia cuando hay algo nuevo o cada 250 ms.
    if (sucia || (long)(ahora - proximoDibujo) >= 0) {
      refrescarPantalla();
      proximoDibujo = ahora + PERIODO_DIBUJO_MS;
      sucia = false;
    }

    // Si la pantalla no arranco (rail lento con fuente externa), reintentar en
    // caliente cada 3 s en vez de quedarse negra para siempre.
    if (!g_pantallaOk && (long)(ahora - proximoReintento) >= 0) {
      proximoReintento = ahora + 3000;
      if (iniciarPantalla(1)) { pitidoOk(); sucia = true; }
    }

    vTaskDelayUntil(&ultima, periodo);
  }
}

// ---------------------------------------------------------------------------
// CONSOLA
// ---------------------------------------------------------------------------
void mostrarEstado() {
  const char *s = g_estadoSensor == SENSOR_OK    ? "OK"
                : g_estadoSensor == SENSOR_FALLA ? "FALLA" : "SIN MEDIDA";
  Serial.println(F("\n--- ESTADO ---"));
  Serial.printf("Salida        : %s\n", g_salidaArmada ? "ARMADA" : "desarmada");
  Serial.printf("Duty comandado: %d %%\n", g_dutyComandado);
  Serial.printf("Duty real     : %.2f %%\n", g_dutyReal);
  Serial.printf("Ventana       : %lu ms\n", (unsigned long)g_ventanaMs);
  Serial.printf("Temperatura   : %.3f C  [%s] %s\n", g_temperatura, s, g_motivoFalla);
  Serial.printf("Setpoint      : %.1f C\n", g_par.setpoint);
  Serial.printf("F0 objetivo   : %.0f min\n", g_par.f0Objetivo);
  Serial.printf("Kp/Ki/Kd      : %.2f / %.3f / %.2f\n", g_par.kp, g_par.ki, g_par.kd);
  Serial.printf("Vista         : %d\n", (int)g_vista);
  if (g_pantallaOk) Serial.printf("Pantalla      : OK en 0x%02X @ %lu Hz\n",
                                  g_dirPantalla, (unsigned long)VELOCIDAD_I2C);
  else              Serial.println(F("Pantalla      : NO DETECTADA"));
  Serial.println();
}

void cmdEncoder() {
  Serial.println(F("\n--- ENCODER: gira y pulsa durante 15 s ---"));
  Serial.println(F("Un click de detente debe dar exactamente 1."));
  int acumulado = 0;
  uint32_t fin = millis() + 15000;
  while ((long)(millis() - fin) < 0) {
    int g = leerGiro();
    if (g) {
      acumulado += g;
      Serial.printf("  giro %+d  -> acumulado %d\n", g, acumulado);
    }
    int p = leerPulsador();
    if (p == 1) Serial.println(F("  pulsacion CORTA"));
    if (p == 2) Serial.println(F("  pulsacion LARGA"));
    delay(20);
  }
  Serial.printf("Total: %d detentes\n\n", acumulado);
}

// Prueba escalonada del buzzer. Distingue buzzer activo de pasivo y separa el
// problema de cableado del de firmware.
void cmdBuzzer() {
  Serial.println();
  Serial.println(F("--- PRUEBA DEL BUZZER (GPIO33) ---"));

  noTone(PIN_BUZZER);
  pinMode(PIN_BUZZER, OUTPUT);

  Serial.println(F("1/5  Nivel alto continuo, 2 s -> suena si el buzzer es ACTIVO"));
  digitalWrite(PIN_BUZZER, HIGH);
  delay(2000);
  digitalWrite(PIN_BUZZER, LOW);
  delay(600);

  Serial.println(F("2/5  Tono de 2 kHz, 2 s -> suena si el buzzer es PASIVO"));
  tone(PIN_BUZZER, 2000);
  delay(2000);
  noTone(PIN_BUZZER);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  delay(600);

  Serial.println(F("3/5  Barrido 500-4000 Hz -> descarta que el tono este fuera"));
  Serial.println(F("     de la frecuencia de resonancia del transductor"));
  for (int f = 500; f <= 4000; f += 100) { tone(PIN_BUZZER, f); delay(40); }
  noTone(PIN_BUZZER);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  delay(600);

  Serial.println(F("4/5  Los tres tonos de la interfaz"));
  pitidoClic();  delay(400);
  pitidoOk();    delay(400);
  pitidoError(); delay(600);

  noTone(PIN_BUZZER);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  Serial.println(F("5/5  Melodia de arranque"));
  melodiaArranque();
  delay(400);

  Serial.println(F("Fin. Si no sono NADA en ningun paso, el fallo es de cableado:"));
  Serial.println(F("  TIP122 de frente -> pin1=BASE, pin2=COLECTOR, pin3=EMISOR"));
  Serial.println(F("  base<-1k<-GPIO33 | colector<-buzzer(-) | emisor<-GND"));
  Serial.println();
}

void ayuda() {
  Serial.println(F("Comandos: estado | armar | desarmar | d <0-100> | enc | buzz | guardar | help"));
}

void procesarComando(String cmd) {
  cmd.trim(); cmd.toLowerCase();
  if (!cmd.length()) return;
  g_ultimoComando = millis();

  if (cmd == "armar") {
    if (g_estadoSensor != SENSOR_OK) {
      Serial.println(F("[!] Sensor sin lectura valida. No se arma."));
      pitidoError();
    } else {
      g_salidaArmada = 1;
      Serial.println(F("*** SALIDA ARMADA ***"));
      pitidoOk();
    }
  }
  else if (cmd == "desarmar") desarmar("comando del operador");
  else if (cmd.startsWith("d ")) {
    int d = cmd.substring(2).toInt();
    if (d > 0 && !g_salidaArmada) { Serial.println(F("[!] Usa 'armar' primero.")); return; }
    g_dutyComandado = constrain(d, 0, 100);
    Serial.printf("Duty -> %d %%\n", g_dutyComandado);
  }
  else if (cmd == "estado")  mostrarEstado();
  else if (cmd == "enc")     cmdEncoder();
  else if (cmd == "buzz")    cmdBuzzer();
  else if (cmd == "guardar") { g_par.ventanaMs = g_ventanaMs; guardarParametros(); }
  else if (cmd == "help")    ayuda();
  else { Serial.printf("Comando desconocido: \"%s\"\n", cmd.c_str()); ayuda(); }
}

// ---------------------------------------------------------------------------

void setup() {
  // SEGURIDAD (§7.2): estado seguro antes que nada.
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, LOW);

  Serial.begin(115200);
  delay(500);

  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);

  Serial.println(F("\n================================================"));
  Serial.println(F(" PID AUTOCLAVE 75X - Paso 3: encoder + OLED"));
  Serial.println(F(" *** SOLO CON LAMPARA / SERIE DE PRUEBA ***"));
  Serial.println(F("================================================"));

  cargarParametros();
  g_ventanaEditable = g_par.ventanaMs;

  termo.begin(MAX31865_3WIRE);
  termo.enable50Hz(FILTRO_50HZ);
  termo.clearFault();

  if (iniciarPantalla(10)) {
    pantalla.clearBuffer();
    pantalla.setFont(u8g2_font_6x10_tf);
    pantalla.drawStr(0, 20, "PID AUTOCLAVE 75X");
    pantalla.drawStr(0, 34, "Iniciando...");
    pantalla.sendBuffer();
  } else {
    // Codigo audible: 3 pitidos graves = pantalla ausente. Sin consola serie
    // conectada es la unica forma de saber por que no hay imagen.
    for (uint8_t i = 0; i < 3; i++) { pitidoError(); delay(350); }
  }

  iniciarEncoder();

  esp_task_wdt_config_t wdt = {};
  wdt.timeout_ms = WDT_TIMEOUT_MS;
  wdt.idle_core_mask = 0;
  wdt.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);

  g_ultimoComando = millis();

  xTaskCreatePinnedToCore(tareaSalida, "salida", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(tareaSensor, "sensor", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(tareaUI,     "ui",     8192, NULL, 1, NULL, 0);

  melodiaArranque();
  Serial.println(F("UI activa. Clic = menu, pulsacion larga = atras."));
  ayuda();
}

void loop() {
  if (Serial.available()) procesarComando(Serial.readStringUntil('\n'));
  delay(20);
}
