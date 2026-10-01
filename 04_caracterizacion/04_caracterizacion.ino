/*
 * 04_caracterizacion.ino
 * PID Autoclave All American 75X - Paso 4 del orden de desarrollo
 *
 * Ensayo de escalon AUTONOMO: el equipo arranca, registra, analiza y muestra
 * resultados sin depender del PC. Corre con su propia alimentacion.
 *
 * *** ESTE SKETCH ENERGIZA LA RESISTENCIA REAL ***
 *
 * Requisitos fisicos previos (§7.1) - ver CHECKLIST_PRIMER_ENSAYO.md:
 *   - Termostato bimetalico NC de 130 C en serie con la fase, aislado por fuera
 *   - Fusible dimensionado en la fase
 *   - SSR sobre disipador con pasta termica
 *   - Valvula de seguridad libre, nivel de agua correcto
 *
 * QUE HACE
 *   Aplica un duty fijo durante N minutos y muestrea la temperatura cada
 *   segundo. Al terminar extrae por el metodo de la tangente:
 *     L    tiempo muerto
 *     tau  constante de tiempo
 *     K    ganancia (C por % de salida)
 *   y propone un PI de arranque por Ziegler-Nichols en lazo abierto.
 *
 * DONDE QUEDAN LOS DATOS
 *   - Buffer en RAM durante el ensayo (2 bytes por muestra)
 *   - Volcado a LittleFS en /ensayo.csv cada 2 min y al terminar, para que un
 *     corte de energia no se lleve la curva entera
 *   - L, tau y K en NVS: sobreviven al reinicio, se ven en "Ver resultados"
 *   - Comando 'volcar' por consola para sacar el CSV cuando haya USB
 *
 * ARQUITECTURA (§8)
 *   tareaSalida  (prio 3,  5 ms)  -> conmuta el SSR, nunca se bloquea
 *   tareaSensor  (prio 2,  1 s)   -> PT100, protecciones y muestreo
 *   tareaUI      (prio 1, 10 ms)  -> encoder; redibuja al cambiar o cada 250 ms
 *   loop()       (prio 1)         -> consola y escritura en flash
 */

#include <Adafruit_MAX31865.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <esp_task_wdt.h>
#include "driver/pulse_cnt.h"
#include <math.h>

// ---------------------------------------------------------------------------
// PINES (§3)
// ---------------------------------------------------------------------------
const uint8_t    PIN_SSR     = 25;
const uint8_t    PIN_CS      = 5;
const gpio_num_t PIN_ENC_CLK = GPIO_NUM_26;
const gpio_num_t PIN_ENC_DT  = GPIO_NUM_27;
const uint8_t    PIN_ENC_SW  = 14;
const uint8_t    PIN_BUZZER  = 33;

// ---------------------------------------------------------------------------
// CONFIGURACION
// ---------------------------------------------------------------------------
const char *VERSION_FW = "1.6.2";
const char *AUTOR_FW   = "BY_Oquendo";

const float   RREF        = 430.0;
const float   RNOMINAL    = 100.0;
const bool    FILTRO_50HZ = false;
// Mediana de 5, no de 3. Medido en el ensayo con carga real: el SSR conmutando
// mete transitorios que triplican el ruido (sd 0.039 -> 0.128 C) con saltos de
// hasta 0.57 C en 1 s. La mediana de 3 solo rechaza 1 muestra corrupta de cada
// 3; la de 5 tolera 2. Cuesta 375 ms por muestra (37 % de un periodo de 1 s),
// que sobra en una planta con constante de tiempo de decenas de minutos.
const uint8_t N_MEDIANA   = 5;

const uint32_t PULSO_MIN_MS   = 20;
const uint32_t WDT_TIMEOUT_MS = 8000;

const uint32_t VELOCIDAD_I2C      = 400000;
const uint8_t  DIRECCIONES_OLED[] = { 0x3C, 0x3D };
const uint32_t PERIODO_SONDEO_MS  = 10;
const uint32_t PERIODO_DIBUJO_MS  = 250;

const int      PASOS_POR_DETENTE  = 4;
const uint32_t PULSACION_LARGA_MS = 600;

// -- Registro --
const uint16_t MAX_MUESTRAS     = 7200;        // 2 h a 1 muestra/s = 14.4 KB
const uint16_t MUESTRAS_FLUSH   = 120;         // volcado a flash cada 2 min
const char    *RUTA_CSV = "/ensayo.csv";

// -- Datos de planta (§1.1 RESUELTO con pinza amperimetrica) --
// Medido: 14 A a 120 VAC = ~1680 W. Escenario B. La lectura de 23.7 ohm en
// frio enganaba; la resistencia real en caliente es ~8.6 ohm.
// Consecuencias: fusible de 20 A, disipador CON ALETAS obligatorio (el SSR
// disipa ~17 W a 14 A), y estos umbrales de marcha en seco recalibrados.
const float POTENCIA_W = 1680.0;
const float CORRIENTE_A = 14.0;

// -- Sitio: Medellin, Parque Explora --
// El punto de ebullicion importa para la deteccion de marcha en seco: la
// meseta de ebullicion ES un estancamiento normal, no una averia (§10.4).
// -- Planta MEDIDA con la olla cerrada (ensayo PID 110 C, 2026-09-09) --
// En el pico del sobrepaso dT/dt = 0, asi que la salida iguala a las perdidas:
//   U = 0.33 x 1680 W / (113.1 - 25) = 6.29 W/K
// Con la tapa ABIERTA U era 19.1 W/K: la evaporacion se llevaba dos tercios.
// Medida en regimen estable durante el ciclo a 121 C (2026-09-09):
//   U = 0.35 x 1680 W / (122.04 - 25) = 6.06 W/K
// El 6.29 anterior salia del pico del sobrepaso, menos fiable que el regimen.
const float U_CERRADA_W_K   = 6.06f;
const float T_AMBIENTE_C    = 25.0f;
const float DUTY_POR_GRADO  = U_CERRADA_W_K / (POTENCIA_W / 100.0f);

// Grados antes del setpoint donde el PID releva a la rampa (§5 Fase 2).
// No es un capricho: si el PID hace toda la subida, satura durante minutos y el
// integrador se engancha al borde de saturacion (I ~ 100 - Kp*error). Al llegar
// al setpoint el integrador vale ~100 cuando el regimen pide 37, y el sobrepaso
// es enorme. Ningun esquema de anti-windup lo evita: acotan el integrador, no
// impiden que tienda al valor que mantiene la salida saturada.
// Entrando cerca del setpoint el error es pequeno, no satura nunca, y la
// precarga hace el resto.
const float ENTREGA_PID_C = 10.0f;

// Banda donde el integrador tiene permiso para acumular.
//
// Durante la aproximacion al setpoint el error es grande y constante durante
// minutos; el integrador acumula aunque la planta, lenta, no haya respondido
// todavia. Medido en el ciclo del 09-09: acumulo ~4.5 puntos de mas en la
// subida de 111 a 121, y eso se tradujo en 1 C de desviacion permanente.
// Congelandolo lejos del objetivo, la precarga y el termino proporcional hacen
// la aproximacion, y el integrador solo pule el ultimo tramo.
const float BANDA_INTEGRAL_C = 2.0f;

const float ALTITUD_M        = 1495.0;
// +/- C alrededor de la ebullicion donde NO se vigila la marcha en seco.
// 5 C cubre de 90 a 100 C: durante una purga larga con algo de contrapresion
// la meseta puede asentarse por encima de los 94.94 C teoricos, y un falso
// positivo ahi abortaria el ciclo justo en la fase mas larga. Por encima de
// 100 C la vigilancia vuelve, que es donde el seco de verdad se manifiesta.
const float BANDA_EBULLICION = 5.0;

// ---------------------------------------------------------------------------
// PROTECCIONES (§7.2)
// ---------------------------------------------------------------------------
const float T_LIMITE_ABSOLUTO = 128.0;
const float T_MIN_VALIDA      = 0.0;
const float T_MAX_VALIDA      = 150.0;

// Deteccion de operacion en seco. Umbrales PROVISIONALES, recalibrar tras §1.1.
// Ver §10.4: durante la meseta de ebullicion el estancamiento es NORMAL. Aqui
// se usa la version simple porque el ensayo es supervisado y acotado.
// Recalibrado para 1680 W: a esa potencia el agua sube ~3.6 C/min al 100 %
// con 4 L y la olla fria, asi que en 5 min deberia subir >10 C. Se exige la
// mitad para dejar margen a perdidas y a cargas grandes.
// Detector LENTO, de respaldo: "potencia a fondo y no pasa nada".
//
// Recalibrado 2026-09-30 tras falsos positivos con carga real. El umbral
// anterior (5 C en 5 min = 1.0 C/min) lo fije con la olla vacia; con 6 kg de
// sustrato la rampa legitima baja a ~0.98 C/min y fallaba por 0.1 C.
//
// Ahora: 10 min y +2 C. Incluso con la carga mas pesada la rampa da mas de
// 5 C en 10 min, y si de verdad no llega calor la temperatura BAJA (delta
// negativo), que es inequivoco. Un detector que da falsos positivos acaba
// puenteado, y entonces no protege de nada.
const int      SECO_DUTY_MINIMO = 80;
const uint32_t SECO_VENTANA_MS  = 10UL * 60UL * 1000UL;
const float    SECO_DELTA_MIN   = 2.0;

// Detector RAPIDO, especifico de la fase de purga.
//
// Con la valvula abierta la temperatura no puede superar la ebullicion
// mientras quede agua: la fisica lo impide, el calor se va en vaporizar. Si
// sube claramente por encima, el agua se acabo. Esto no es una heuristica,
// es una imposibilidad termodinamica, y actua en segundos en vez de minutos.
//
// Medido en purga real: ~97 C, o sea 2 C de contrapresion por el vapor
// saliendo. El umbral en +8 C deja margen de sobra.
const float SECO_SOBRE_EBU_C = 8.0f;

// -- Gracia y antirrebote --
// Al encender, el MAX31865 reporta flags espurios mientras se estabilizan la
// alimentacion y la corriente de bias; con fuente externa el rail sube mas
// despacio que por USB y el flag OVUV es habitual. Y un unico flag transitorio
// no debe latchar un estado terminal.
const uint8_t GRACIA_ARRANQUE_S  = 4;   // s de arranque en que se ignoran flags
const uint8_t DEBOUNCE_SENSOR    = 3;   // lecturas seguidas para actuar
const uint8_t DEBOUNCE_SOBRETEMP = 2;

// Cambio de temperatura fisicamente imposible entre dos muestras de 1 s: en
// esta planta la variacion real no pasa de ~0.1 C/s ni en la rampa mas viva.
// Un salto mayor delata contacto intermitente o ruido, no fisica.
const float   SALTO_MAX_C_S       = 5.0;

// IIR sobre la mediana, para lo que el rechazo de picos no alcanza. Con alfa
// 0.30 a 1 Hz la constante de tiempo del filtro es ~3 s: irrelevante frente a
// los ~900 s de la planta, asi que no introduce retraso de control apreciable.
const float   ALFA_IIR            = 0.30;
// Lecturas seguidas con flag activo para considerarlo real y no un artefacto
// del ciclo de deteccion (ver comentario largo sobre tareaSensor).
const uint8_t FLAGS_PARA_INVALIDAR = 5;

// ---------------------------------------------------------------------------
// ESTADO
// ---------------------------------------------------------------------------
enum Estado : int { EST_IDLE = 0, EST_ENSAYO = 1, EST_FALLA = 2, EST_PID = 3, EST_CICLO = 4 };
volatile int g_estado = EST_IDLE;
char g_motivoFalla[48] = "";
bool g_fallaCritica    = false;   // critica = no se borra desde el encoder
volatile bool g_pedirMelodiaFin = false;  // F0 cumplido: lo suena loop()
char g_avisoSensor[32] = "";   // problema actual del sensor, sin latchar

volatile int   g_dutyComandado = 0;
volatile int   g_salidaArmada  = 0;
volatile float g_dutyReal      = 0.0f;
volatile float g_temperatura   = NAN;
volatile int   g_sensorValido  = 0;

volatile uint32_t g_inicioEnsayo     = 0;
volatile uint32_t g_duracionEnsayoMs = 0;
volatile int      g_dutyEnsayo       = 0;
volatile float    g_tInicioEnsayo    = NAN;

// Muestras en centigrados x100: entero, compacto, resolucion de 0.01 C
int16_t  g_muestras[MAX_MUESTRAS];
volatile uint16_t g_nMuestras  = 0;
volatile bool     g_pedirFlush = false;

struct Fopdt { float L, tau, K, pendiente; bool valido; };
Fopdt g_res = { 0, 0, 0, 0, false };

struct Parametros {
  float setpoint, f0Objetivo, kp, ki, kd;
  float dutyEnsayo, minutosEnsayo;
  float offsetC;               // correccion contra el patron de laboratorio
  uint32_t ventanaMs;
} g_par;

// Lectura SIN corregir, para el ajuste de calibracion y como respaldo de las
// protecciones: un offset mal puesto no debe poder enmascarar una sobretemp.
volatile float g_tCruda = NAN;

// Limite del offset. Una PT100 clase A con Rref correcta no se desvia mas de
// esto a 121 C; un error mayor significa que hay un problema real que tapar
// con un offset seria peor que dejarlo visible.
const float OFFSET_MAX_C = 3.0;

Preferences prefs;
Adafruit_MAX31865 termo(PIN_CS);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C pantalla(U8G2_R0, U8X8_PIN_NONE);
pcnt_unit_handle_t pcntUnidad = NULL;

volatile bool     g_pantallaOk  = false;
uint8_t           g_dirPantalla = 0;
volatile uint32_t g_ventanaMs   = 2000;

// Punto de ebullicion corregido por altitud. En Medellin son 94.94 C, no 100.
float puntoEbullicion() {
  float p_kPa = 101.325f * powf(1.0f - 2.25577e-5f * ALTITUD_M, 5.25588f);
  return 1.0f / (1.0f / 373.15f - 8.314f / 40660.0f * logf(p_kPa / 101.325f)) - 273.15f;
}

// ---------------------------------------------------------------------------
// NVS
// ---------------------------------------------------------------------------
void cargarParametros() {
  prefs.begin("autoclave", false);
  g_par.setpoint      = prefs.getFloat("sp",   121.0f);
  g_par.f0Objetivo    = prefs.getFloat("f0",    20.0f);
  g_par.kp            = prefs.getFloat("kp",     2.0f);
  g_par.ki            = prefs.getFloat("ki",     0.05f);
  g_par.kd            = prefs.getFloat("kd",     0.0f);
  g_par.dutyEnsayo    = prefs.getFloat("dut",   50.0f);
  g_par.minutosEnsayo = prefs.getFloat("min",   45.0f);
  g_par.offsetC       = prefs.getFloat("off",    0.0f);
  g_par.ventanaMs     = prefs.getUInt ("vent", 2000);
  g_ventanaMs = g_par.ventanaMs;

  g_res.L      = prefs.getFloat("rL",   0.0f);
  g_res.tau    = prefs.getFloat("rTau", 0.0f);
  g_res.K      = prefs.getFloat("rK",   0.0f);
  g_res.valido = prefs.getBool ("rOk",  false);
}

void guardarParametros() {
  prefs.putFloat("sp",   g_par.setpoint);
  prefs.putFloat("f0",   g_par.f0Objetivo);
  prefs.putFloat("kp",   g_par.kp);
  prefs.putFloat("ki",   g_par.ki);
  prefs.putFloat("kd",   g_par.kd);
  prefs.putFloat("dut",  g_par.dutyEnsayo);
  prefs.putFloat("min",  g_par.minutosEnsayo);
  prefs.putFloat("off",  g_par.offsetC);
  prefs.putUInt ("vent", g_par.ventanaMs);
  Serial.println(F("# Parametros guardados en NVS."));
}

void guardarResultados() {
  prefs.putFloat("rL",   g_res.L);
  prefs.putFloat("rTau", g_res.tau);
  prefs.putFloat("rK",   g_res.K);
  prefs.putBool ("rOk",  g_res.valido);
}

// ---------------------------------------------------------------------------
// BUZZER (etapa TIP122, §3.3)
// ---------------------------------------------------------------------------
void pitidoClic()  { tone(PIN_BUZZER, 2000,  15); }
void pitidoOk()    { tone(PIN_BUZZER, 1500,  60); }
void pitidoAviso() { tone(PIN_BUZZER, 1000, 150); }
void pitidoFalla() { for (uint8_t i = 0; i < 5; i++) { tone(PIN_BUZZER, 400, 200); delay(280); } }

// Arpegio ascendente que remata arriba: las notas agudas caen cerca de la
// resonancia tipica de los transductores piezo (2-4 kHz) y suenan mas fuerte.
void melodiaArranque() {
  struct Nota { uint16_t freq, ms; };
  static const Nota MELODIA[] = {
    {  784,  70 }, { 1047,  70 }, { 1319,  70 }, { 1568,  70 }, { 2093, 320 },
  };
  for (uint8_t i = 0; i < sizeof(MELODIA) / sizeof(Nota); i++) {
    tone(PIN_BUZZER, MELODIA[i].freq, MELODIA[i].ms);
    delay(MELODIA[i].ms + 25);
  }
  noTone(PIN_BUZZER);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
}

void melodiaFin() {                       // ensayo terminado correctamente
  const uint16_t F[] = { 1568, 1319, 1568, 2093 };
  for (uint8_t i = 0; i < 4; i++) { tone(PIN_BUZZER, F[i], 140); delay(170); }
  noTone(PIN_BUZZER);
}

// ---------------------------------------------------------------------------
// SEGURIDAD
// ---------------------------------------------------------------------------
void ponerSSR(bool on) { digitalWrite(PIN_SSR, on ? HIGH : LOW); }

// El estado de FALLA es terminal: solo se sale con reset manual (§7.2).
// Dos niveles de falla, porque las causas no son igual de graves (7.2).
//
//   RECUPERABLE: marcha en seco, tiempo maximo agotado. La causa esta afuera
//   del equipo -- falta agua, la carga es mas grande de lo previsto -- y el
//   operador la corrige y vuelve a arrancar. Se reconoce con pulsacion larga.
//
//   CRITICA: la temperatura paso el limite absoluto, o el sensor quedo ciego
//   con la salida energizada. Eso significa que fallo el hardware: el SSR
//   conduce sin orden, o el control esta a ciegas. El ciclo siguiente
//   calentaria sin nadie vigilando, asi que NO se borra desde el panel y
//   sobrevive al corte de alimentacion. Solo sale con "reset" por consola,
//   cuando alguien ya reviso que paso.
void latcharFalla(const char *motivo, bool critica) {
  ponerSSR(false);
  g_dutyComandado = 0;
  g_salidaArmada  = 0;
  g_estado        = EST_FALLA;
  g_fallaCritica  = critica;
  strncpy(g_motivoFalla, motivo, sizeof(g_motivoFalla) - 1);
  g_motivoFalla[sizeof(g_motivoFalla) - 1] = '\0';
  g_pedirFlush = true;                    // conservar lo registrado hasta aqui
  if (critica) {
    prefs.putBool  ("fcrit", true);
    prefs.putString("fmot",  g_motivoFalla);
  }
  Serial.println();
  Serial.println(F("# ***************************************************"));
  Serial.printf ("# *** FALLA%s: %s\n", critica ? " CRITICA" : "", motivo);
  if (critica)
    Serial.println(F("# *** Grabada en NVS. Apagar el equipo NO la borra. ***"));
  Serial.println(F("# *** Salida cortada. Requiere reset del operador. ***"));
  Serial.println(F("# ***************************************************"));
}

void entrarEnFalla(const char *motivo)        { latcharFalla(motivo, false); }
void entrarEnFallaCritica(const char *motivo) { latcharFalla(motivo, true);  }

// Se llama UNA vez en setup(), no desde cargarParametros(): recargar los
// parametros desde el menu no debe resucitar una falla ya reconocida.
void restaurarFallaCritica() {
  if (!prefs.getBool("fcrit", false)) return;
  String m = prefs.getString("fmot", "falla critica previa al reinicio");
  strncpy(g_motivoFalla, m.c_str(), sizeof(g_motivoFalla) - 1);
  g_motivoFalla[sizeof(g_motivoFalla) - 1] = '\0';
  g_fallaCritica = true;
  g_estado       = EST_FALLA;
  g_salidaArmada = 0;
  Serial.println(F("# *** ARRANCA EN FALLA CRITICA SIN RECONOCER ***"));
  Serial.printf ("# *** Motivo guardado: %s\n", g_motivoFalla);
  Serial.println(F("# *** Revise el SSR y el sensor antes de dar reset. ***"));
}

void desarmar(const char *motivo) {
  g_salidaArmada  = 0;
  g_dutyComandado = 0;
  g_dutyReal      = 0.0f;
  ponerSSR(false);
  Serial.printf("# Salida desarmada: %s\n", motivo);
}

// ---------------------------------------------------------------------------
// CONTROLADOR PI (§6)
//
// Solo PI, sin derivativo: el documento es explicito en que el termino D en un
// proceso con vapor y cambio de fase genera mas problemas que soluciones.
//
// Anti-windup por integracion condicional, que §6 marca como obligatorio. Sin
// el, durante la rampa larga hasta el setpoint el integrador acumula un error
// enorme y luego provoca un sobrepaso brutal, que en un recipiente a presion es
// sobrepresion. Aqui el integrador solo acumula si la salida no esta saturada,
// o si el error empuja de vuelta hacia el rango util.
// ---------------------------------------------------------------------------
float g_integral   = 0.0f;
bool  g_enRampa    = false;   // true = 100 % abierto, aun no releva el PID
float g_errorPID   = 0.0f;
float g_salidaPID  = 0.0f;

// Arranca el lazo con el integrador PRECARGADO al duty de regimen estimado.
//
// Por que. En el setpoint el termino proporcional vale cero, asi que toda la
// potencia de mantenimiento tiene que salir del integrador. Arrancando en cero,
// el integrador debe recorrer ese camino entero, y en una planta lenta (tau de
// 84 min con la olla cerrada) se pasa de largo antes de que la temperatura
// responda. Medido en el ensayo del 2026-09-09: el integrador llego a 49 cuando
// el regimen pedia 32, y el sobrepaso fue de +3.13 C sobre un setpoint de 110.
//
// Precargando, el lazo empieza equilibrado y solo corrige la diferencia. Es lo
// mismo que hace una transferencia sin salto de manual a automatico en un
// controlador industrial, y encaja con la seccion 5: la rampa lleva la olla
// hasta 105-110 C y el PID entra ya cargado, no desde cero.
void reiniciarPID(float setpoint) {
  float regimen = DUTY_POR_GRADO * (setpoint - T_AMBIENTE_C);
  if (regimen < 0.0f)   regimen = 0.0f;
  if (regimen > 100.0f) regimen = 100.0f;
  g_integral  = regimen;
  g_errorPID  = 0.0f;
  g_salidaPID = regimen;
  Serial.printf("# PID: integrador precargado a %.1f %% (regimen para %.1f C)\n",
                regimen, setpoint);
}

float calcularPI(float sp, float pv, float dt) {
  float error   = sp - pv;
  float propor  = g_par.kp * error;

  // El integrador solo acumula cerca del objetivo (ver BANDA_INTEGRAL_C)
  bool  puedeIntegrar = fabsf(error) < BANDA_INTEGRAL_C;
  float integTent = puedeIntegrar ? g_integral + g_par.ki * error * dt : g_integral;
  float salida  = propor + integTent;

  if (salida > 100.0f) {
    salida = 100.0f;
    if (error < 0) g_integral = integTent;      // deja descargar el integrador
  } else if (salida < 0.0f) {
    salida = 0.0f;
    if (error > 0) g_integral = integTent;
  } else {
    g_integral = integTent;
  }

  g_errorPID  = error;
  g_salidaPID = salida;
  return salida;
}

// ---------------------------------------------------------------------------
// CICLO DE ESTERILIZACION (§5)
//
// Secuencia completa, con el operador guiado paso a paso:
//
//   CALENTANDO   valvula abierta, 100 %, hasta la ebullicion
//   PURGA        7 min de vapor continuo barriendo el aire
//   CERRAR       aviso acustico: cierre la valvula
//   RAMPA        100 % hasta setpoint - 10, el PID releva ahi
//   MESETA       PID + acumulacion de F0 hasta el objetivo
//   TERMINADO    salida cortada, no abrir hasta presion cero
//
// La purga no es opcional: es el modo de falla numero uno en autoclaves
// domesticas. Si queda aire atrapado se forman bolsas donde el vapor no
// penetra, la temperatura local queda muy por debajo de la consigna, y la
// PT100 del termowell no se entera de nada.
// ---------------------------------------------------------------------------
enum Fase : uint8_t {
  F_CALENTANDO = 0, F_PURGA, F_CERRAR_VALVULA, F_RAMPA, F_MESETA,
  F_TERMINADO,      // ciclo cumplido, enfriando, aun con presion
  F_LISTO_ABRIR     // por debajo del umbral de despresurizacion
};
volatile uint8_t g_fase = F_CALENTANDO;

// Duracion de la purga: FIJA en 7 minutos, deliberadamente NO ajustable.
//
// Para la camara sola sobra: a 1680 W se generan 1.55 L/s de vapor y el volumen
// libre (~23 L) se renueva cada 15 s, o sea 28 veces en 7 min. Lo que de verdad
// cuesta desplazar es el aire atrapado en los poros de la carga, y eso no se
// calcula: depende del empaque y la humedad.
//
// Ademas la valvula esta en la TAPA, no abajo como en un desplazamiento por
// gravedad bien hecho, y el estandar para estos recipientes son 10 min.
//
// Se deja fija a proposito: la purga insuficiente no falla de forma visible
// -deja bolsas de aire que la PT100 del termowell no ve- y solo un indicador
// biologico en el centro de la carga puede decir si basta (§10.17, §10.18).
// No es un parametro para tocar a ojo.
const uint32_t PURGA_MS = 7UL * 60UL * 1000UL;
const float    MARGEN_EBU_C     = 1.0f;   // cuanto antes de la ebullicion contar
const float    SUBIDA_CIERRE_C  = 2.0f;   // subida que demuestra valvula cerrada
const uint32_t AVISO_CIERRE_MS  = 5UL * 60UL * 1000UL;  // reaviso si no sube

// Umbral por debajo del cual la presion manometrica es cero con margen.
//
// NO son 100 C: eso es la ebullicion a nivel del mar. En Medellin el agua
// hierve a 94.94 C, asi que a 100 C todavia quedarian 2.4 psi. Se usan 90 C
// para dejar ~5 C de margen frente al error del sensor (+/-0.3 C), el retardo
// del termowell y la variacion barometrica del dia (+/-0.4 C).
//
// El aviso NO autoriza a abrir: la temperatura es una inferencia, el manometro
// es una medida directa. Si la sonda fallara leyendo bajo, el firmware diria
// "seguro" con presion dentro. El mensaje pide verificar el manometro.
const float    T_DESPRESURIZADO_C = 90.0f;
// Reaviso de "ya se puede abrir" en dos ritmos. Los primeros minutos cada
// minuto, porque es cuando el operador probablemente anda cerca. Despues
// mas espaciado: una alarma que suena cada minuto toda la noche termina
// con el buzzer desconectado, y entonces no queda alarma para nada.
const uint32_t REAVISO_ABRIR_MS   = 60UL * 1000UL;
const uint32_t REAVISO_LENTO_MS   = 5UL * 60UL * 1000UL;
const uint32_t REAVISO_INSISTE_MS = 10UL * 60UL * 1000UL;  // tramo rapido

volatile uint32_t g_inicioFase   = 0;

// Cronometro del ciclo completo. Se congela al completarse la esterilizacion:
// el tiempo que interesa registrar es el del proceso, no el del enfriamiento.
volatile uint32_t g_finCiclo     = 0;   // 0 = en curso

uint32_t segundosCiclo() {
  uint32_t fin = g_finCiclo ? g_finCiclo : millis();
  return (fin - g_inicioEnsayo) / 1000;
}
volatile float    g_F0           = 0.0f;
volatile float    g_tAlCerrar    = NAN;   // T cuando se pidio cerrar la valvula
volatile uint32_t g_ultimoAviso  = 0;
volatile uint8_t  g_avisosCierre = 0;

const char *nombreFase(uint8_t f) {
  switch (f) {
    case F_CALENTANDO:     return "CALENTANDO";
    case F_PURGA:          return "PURGA";
    case F_CERRAR_VALVULA: return "CERRAR VALVULA";
    case F_RAMPA:          return "PRESURIZANDO";
    case F_MESETA:         return "ESTERILIZANDO";
    case F_TERMINADO:      return "ENFRIANDO";
    default:               return "DESPRESURIZADO";
  }
}

void cambiarFase(uint8_t nueva) {
  g_fase = nueva;
  g_inicioFase = millis();
  if (nueva == F_TERMINADO) g_finCiclo = millis();   // detiene el cronometro
  Serial.printf("# FASE -> %s  (T=%.2f C)\n", nombreFase(nueva), g_temperatura);
}

// Aviso acustico insistente para pedir accion al operador
void avisoAccion() {
  for (uint8_t i = 0; i < 3; i++) {
    tone(PIN_BUZZER, 2093, 180); delay(210);
    tone(PIN_BUZZER, 1568, 180); delay(210);
  }
  noTone(PIN_BUZZER);
}

// Melodia de fin de ciclo: agradable, distinta de la de falla
void melodiaFinCiclo() {
  const uint16_t F[] = { 1047, 1319, 1568, 2093, 1568, 2093 };
  const uint16_t D[] = {  160,  160,  160,  220,  160,  420 };
  for (uint8_t i = 0; i < 6; i++) { tone(PIN_BUZZER, F[i], D[i]); delay(D[i] + 40); }
  noTone(PIN_BUZZER);
}

// Avanza la maquina de estados. Se llama una vez por segundo desde el sensor.
// Devuelve el duty que corresponde a la fase actual.
int avanzarCiclo(float t) {
  uint32_t enFase = millis() - g_inicioFase;
  float    ebu    = puntoEbullicion();

  switch (g_fase) {

    case F_CALENTANDO:
      // Valvula abierta. Se calienta a fondo hasta que rompe a hervir.
      if (t > ebu + SECO_SOBRE_EBU_C) {
        char b[56];
        snprintf(b, sizeof(b), "SECO: %.1f C con valvula abierta", t);
        entrarEnFalla(b);
        return 0;
      }
      if (t >= ebu - MARGEN_EBU_C) { cambiarFase(F_PURGA); avisoAccion(); }
      return 100;

    case F_PURGA:
      // Siete minutos de vapor continuo. Si la temperatura cae por debajo de
      // la ebullicion es que dejo de hervir: se vuelve a calentar y el conteo
      // se reinicia, porque una purga interrumpida no barre el aire.
      // Con la valvula abierta y agua dentro, la temperatura se queda en la
      // ebullicion. Que suba claramente por encima solo puede significar que
      // ya no hay agua que vaporizar.
      if (t > ebu + SECO_SOBRE_EBU_C) {
        char b[56];
        snprintf(b, sizeof(b), "SECO: %.1f C purgando con valvula abierta", t);
        entrarEnFalla(b);
        return 0;
      }
      if (t < ebu - MARGEN_EBU_C - 1.0f) { cambiarFase(F_CALENTANDO); return 100; }
      if (enFase >= PURGA_MS) {
        g_tAlCerrar = t;
        g_avisosCierre = 0;
        cambiarFase(F_CERRAR_VALVULA);
        avisoAccion();
      }
      return 100;

    case F_CERRAR_VALVULA:
      // Se pide cerrar la valvula. La confirmacion no la da un boton: la da la
      // fisica. Con la olla sellada la temperatura supera la ebullicion; si no
      // sube, la valvula sigue abierta y se vuelve a avisar.
      if (!isnan(g_tAlCerrar) && t >= g_tAlCerrar + SUBIDA_CIERRE_C) {
        cambiarFase(F_RAMPA);
        pitidoOk();
      } else if (enFase - g_ultimoAviso >= AVISO_CIERRE_MS) {
        g_ultimoAviso = enFase;
        g_avisosCierre++;
        Serial.printf("# [!] La temperatura no sube: la valvula sigue abierta? (aviso %u)\n",
                      g_avisosCierre);
        avisoAccion();
      }
      return 100;

    case F_RAMPA:
      // Lazo abierto hasta cerca del setpoint; ahi releva el PID precargado.
      if (t >= g_par.setpoint - ENTREGA_PID_C) {
        reiniciarPID(g_par.setpoint);
        cambiarFase(F_MESETA);
        pitidoOk();
        return (int)lroundf(g_salidaPID);
      }
      return 100;

    case F_TERMINADO:
      // Ciclo cumplido y salida cortada. Se sigue vigilando el enfriamiento
      // para avisar cuando la presion haya caido a cero.
      if (t < T_DESPRESURIZADO_C) {
        cambiarFase(F_LISTO_ABRIR);
        g_ultimoAviso = 0;
        avisoAccion();
      }
      return 0;

    case F_LISTO_ABRIR: {
      // Reaviso periodico: el operador puede no estar delante. No se calla
      // nunca -- la carga no debe quedar olvidada -- pero pasa a un ritmo
      // mas lento una vez que es evidente que no hay nadie cerca.
      uint32_t cada = (enFase < REAVISO_INSISTE_MS) ? REAVISO_ABRIR_MS
                                                    : REAVISO_LENTO_MS;
      if (enFase - g_ultimoAviso >= cada) {
        g_ultimoAviso = enFase;
        avisoAccion();
      }
      return 0;
    }

    case F_MESETA: {
      // Acumulacion de letalidad (§5 Fase 3). z = 10 C, referencia 121.1 C.
      // Las caidas breves se integran solas: no hay conteo que reiniciar.
      g_F0 += powf(10.0f, (t - 121.1f) / 10.0f) / 60.0f;   // dt = 1 s
      if (g_F0 >= g_par.f0Objetivo) {
        cambiarFase(F_TERMINADO);
        // La melodia la toca loop(): dura 1.3 s y esta tarea no puede
        // quedarse bloqueada tanto tiempo sin leer la temperatura.
        g_pedirMelodiaFin = true;
        Serial.printf("# ESTERILIZACION COMPLETA. F0 = %.2f min\n", g_F0);
        return 0;
      }
      return (int)lroundf(calcularPI(g_par.setpoint, t, 1.0f));
    }

    default:
      return 0;
  }
}

// ---------------------------------------------------------------------------
// TAREA DE SALIDA
// ---------------------------------------------------------------------------
void tareaSalida(void *pv) {
  esp_task_wdt_add(NULL);
  const TickType_t periodo = pdMS_TO_TICKS(5);
  TickType_t ultima = xTaskGetTickCount();

  uint32_t inicioVentana = millis();
  uint32_t onAcumulado = 0, marcaOn = 0;
  bool estadoActual = false;

  for (;;) {
    esp_task_wdt_reset();
    uint32_t ahora   = millis();
    uint32_t ventana = g_ventanaMs;

    if ((uint32_t)(ahora - inicioVentana) >= ventana) {
      if (estadoActual) { onAcumulado += ahora - marcaOn; marcaOn = ahora; }
      g_dutyReal = 100.0f * onAcumulado / (float)ventana;
      onAcumulado = 0;
      if ((uint32_t)(ahora - inicioVentana) > 2 * ventana) inicioVentana = ahora;
      else                                                inicioVentana += ventana;
    }

    bool deseado = false;
    if (g_estado != EST_FALLA && g_salidaArmada && g_dutyComandado > 0) {
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
// ANALISIS FOPDT - metodo de la tangente
// ---------------------------------------------------------------------------
Fopdt analizarCurva() {
  Fopdt r = { 0, 0, 0, 0, false };
  uint16_t n = g_nMuestras;
  if (n < 30 || g_dutyEnsayo <= 0) return r;

  float T0 = g_muestras[0]     / 100.0f;
  float Tf = g_muestras[n - 1] / 100.0f;
  float dT = Tf - T0;
  if (dT < 5.0f) return r;                 // no subio lo suficiente

  // Pendiente maxima sobre ventana movil de 30 s (las muestras van a 1 Hz)
  const uint16_t V = 30;
  if (n <= V) return r;
  float mejor = 0.0f;
  uint16_t iMejor = 0;
  for (uint16_t i = 0; i + V < n; i++) {
    float p = (g_muestras[i + V] - g_muestras[i]) / 100.0f / (float)V;
    if (p > mejor) { mejor = p; iMejor = i; }
  }
  if (mejor <= 0.0f) return r;

  // La tangente en el punto de inflexion corta T0 en L, y Tf en L + tau
  float tInfl = iMejor + V / 2.0f;
  float TInfl = g_muestras[iMejor + V / 2] / 100.0f;

  r.pendiente = mejor;
  r.L = tInfl - (TInfl - T0) / mejor;
  if (r.L < 0) r.L = 0;
  r.tau = (Tf - T0) / mejor;
  r.K   = dT / (float)g_dutyEnsayo;
  r.valido = true;
  return r;
}

void imprimirResultados(const Fopdt &r) {
  Serial.println(F("# ========================================="));
  Serial.println(F("#  ANALISIS DE LA RESPUESTA AL ESCALON"));
  Serial.println(F("# ========================================="));
  Serial.printf("# Duty aplicado   : %d %%\n", g_dutyEnsayo);
  Serial.printf("# Muestras        : %u  (%.1f min)\n", g_nMuestras, g_nMuestras / 60.0);
  if (g_nMuestras)
    Serial.printf("# T inicial/final : %.2f / %.2f C\n",
                  g_muestras[0] / 100.0, g_muestras[g_nMuestras - 1] / 100.0);

  if (!r.valido) {
    Serial.println(F("# Curva insuficiente para identificar la planta."));
    Serial.println(F("# Repetir con mas salida o mas tiempo."));
    Serial.println(F("# ========================================="));
    return;
  }
  Serial.printf("# Pendiente max   : %.3f C/min\n", r.pendiente * 60.0);
  Serial.printf("# L   (t. muerto) : %.1f s  (%.2f min)\n", r.L, r.L / 60.0);
  Serial.printf("# tau (cte tiempo): %.1f s  (%.2f min)\n", r.tau, r.tau / 60.0);
  Serial.printf("# K   (ganancia)  : %.4f C por %%\n", r.K);
  if (r.L > 0) Serial.printf("# tau/L           : %.2f\n", r.tau / r.L);

  if (r.L > 0 && r.K > 0) {
    float kp = 0.9f * r.tau / (r.K * r.L);
    float ti = 3.33f * r.L;
    Serial.println(F("# PI de arranque (Ziegler-Nichols, lazo abierto):"));
    Serial.printf("#   Kp = %.3f   Ki = %.5f   Kd = 0\n", kp, kp / ti);
    Serial.println(F("# Valores de partida, no finales. Refinar con sTune"));
    Serial.println(F("# en la zona de 110-121 C, nunca desde ambiente."));
  }
  Serial.println(F("# ========================================="));
}

// ---------------------------------------------------------------------------
// PERSISTENCIA EN FLASH
// ---------------------------------------------------------------------------
void escribirCSV() {
  File f = LittleFS.open(RUTA_CSV, "w");
  if (!f) { Serial.println(F("# [X] No se pudo abrir /ensayo.csv")); return; }
  f.printf("# duty=%d muestras=%u periodo=1s\n", g_dutyEnsayo, g_nMuestras);
  f.println("t_s,T_C");
  for (uint16_t i = 0; i < g_nMuestras; i++) f.printf("%u,%.2f\n", i, g_muestras[i] / 100.0);
  f.close();
  Serial.printf("# Curva guardada en %s (%u muestras)\n", RUTA_CSV, g_nMuestras);
}

void volcarCSV() {
  File f = LittleFS.open(RUTA_CSV, "r");
  if (!f) { Serial.println(F("# No hay ningun ensayo guardado.")); return; }
  Serial.println(F("# ---- INICIO VOLCADO ----"));
  while (f.available()) Serial.write(f.read());
  f.close();
  Serial.println(F("# ---- FIN VOLCADO ----"));
}

// ---------------------------------------------------------------------------
// SENSOR + PROTECCIONES + MUESTREO
// ---------------------------------------------------------------------------
inline float rawAGrados(uint16_t raw) { return termo.calculateTemperature(raw, RNOMINAL, RREF); }

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

void terminarEnsayo(const char *motivo) {
  bool eraEscalon = (g_estado == EST_ENSAYO);
  g_dutyComandado = 0;
  g_salidaArmada  = 0;
  ponerSSR(false);
  g_estado = EST_IDLE;
  // El analisis FOPDT solo tiene sentido sobre un escalon de duty fijo. En
  // modo PID la salida varia, asi que la curva no sirve para identificar.
  if (eraEscalon) { g_res = analizarCurva(); guardarResultados(); }
  g_pedirFlush = true;
  Serial.printf("# ENSAYO TERMINADO: %s\n", motivo);
  imprimirResultados(g_res);
}

// Traduce el registro de fallas del MAX31865 a texto corto para la pantalla.
const char *textoFalla(uint8_t f) {
  if (f & MAX31865_FAULT_HIGHTHRESH) return "RTD sobre umbral alto";
  if (f & MAX31865_FAULT_LOWTHRESH)  return "RTD bajo umbral bajo";
  if (f & MAX31865_FAULT_REFINLOW)   return "REFIN- alto: RTD abierta";
  if (f & MAX31865_FAULT_REFINHIGH)  return "REFIN- bajo: FORCE- abierto";
  if (f & MAX31865_FAULT_RTDINLOW)   return "RTDIN- bajo: FORCE- abierto";
  if (f & MAX31865_FAULT_OVUV)       return "sobre/subtension del bus";
  return "flag desconocido";
}

/*
 * VALIDEZ DEL SENSOR: se decide por la LECTURA, no por el registro de fallas.
 *
 * Por que. El ciclo automatico de deteccion de fallas del MAX31865 solo espera
 * 1 ms a que asiente la corriente de bias (asi lo implementa la libreria). Con
 * el cable largo de la instalacion definitiva esa espera se queda corta y el
 * chip levanta flags REFIN-/RTDIN- de forma intermitente aunque todo funcione:
 * medido en banco, tres lecturas seguidas dieron 0x18, limpio y 0x0C mientras
 * la resistencia se mantenia en 109.95-110.03 ohm (0.2 C de dispersion).
 *
 * Esto NO debilita la proteccion, porque los modos de fallo reales se ven en la
 * propia lectura:
 *     RTD abierta            -> raw saturado
 *     RTD en corto           -> raw ~ 0
 *     FORCE- abierto         -> sin corriente, raw ~ 0
 *     contacto intermitente  -> saltos fisicamente imposibles entre muestras
 * El unico modo que no cubre es la deriva de calibracion, que tampoco detecta
 * el registro de fallas: para eso esta la comparacion con el patron de
 * laboratorio (§10.1).
 *
 * El registro de fallas se conserva como diagnostico y como corroboracion: si
 * persiste muchas lecturas seguidas, tambien invalida.
 */
void tareaSensor(void *pv) {
  const TickType_t periodo = pdMS_TO_TICKS(1000);
  TickType_t ultima = xTaskGetTickCount();

  uint32_t marcaSeco  = millis();
  float    tMarcaSeco = NAN;
  float    tAnterior  = NAN;

  uint8_t ciclosGracia    = GRACIA_ARRANQUE_S;
  uint8_t fallosSeguidos  = 0;
  uint8_t sobreTempSegs   = 0;
  uint8_t flagsSeguidos   = 0;

  for (;;) {
    uint16_t raw = leerRawMediana();
    float    tCruda = rawAGrados(raw);

    // IIR encadenado a la mediana. Se arranca con el primer valor valido para
    // que no haya rampa artificial al inicio del ensayo.
    static float tFiltrada = NAN;
    float t = tCruda;
    if (!isnan(tCruda)) {
      if (isnan(tFiltrada)) tFiltrada = tCruda;
      else                  tFiltrada = ALFA_IIR * tCruda + (1.0f - ALFA_IIR) * tFiltrada;
      t = tFiltrada;
    }
    g_tCruda = t;                       // filtrada pero SIN corregir
    t += g_par.offsetC;                 // temperatura de proceso, corregida

    uint8_t flags = termo.readFault();
    if (flags) { termo.clearFault(); flagsSeguidos++; } else flagsSeguidos = 0;

    // -- Validez por la lectura --
    const char *problema = NULL;
    if      (raw == 0)                    problema = "RTD en corto o sin corriente";
    else if (raw >= 32767)                problema = "RTD abierta";
    else if (isnan(t))                    problema = "lectura NaN";
    else if (t < T_MIN_VALIDA)            problema = "T por debajo de 0 C";
    else if (t > T_MAX_VALIDA)            problema = "T por encima de 150 C";
    else if (!isnan(tAnterior) && fabsf(t - tAnterior) > SALTO_MAX_C_S)
                                          problema = "salto imposible entre muestras";
    // El registro de fallas NO invalida la lectura. Medido en el ensayo con
    // carga real: el SSR conmutando 5-14 A junto al cable de la PT100 dispara
    // el ciclo de deteccion de fallas del MAX31865 aunque la medida sea buena.
    // Abortar un ensayo valido por eso es peor que no tener el flag: lleva a
    // que el operador acabe puenteando la proteccion. Queda como diagnostico.

    g_temperatura  = t;
    g_sensorValido = problema ? 0 : 1;
    tAnterior      = isnan(t) ? tAnterior : t;

    if (problema) snprintf(g_avisoSensor, sizeof(g_avisoSensor), "%s", problema);
    else          g_avisoSensor[0] = '\0';

    // Flag persistente: se avisa una vez, sin invalidar ni abortar nada.
    if (flagsSeguidos == FLAGS_PARA_INVALIDAR)
      Serial.printf("# [i] Flag persistente 0x%02X (%s). Lectura OK, se ignora.\n",
                    flags, textoFalla(flags));

    // -- Periodo de gracia --
    // Al encender, la alimentacion y el bias tardan en asentarse. Se informa
    // pero no se actua: en esos primeros segundos no hay nada energizado.
    if (ciclosGracia) {
      ciclosGracia--;
      if (problema) Serial.printf("# Arranque: %s (ignorado)\n", problema);
      vTaskDelayUntil(&ultima, periodo);
      continue;
    }

    // -- Antirrebote --
    // La mediana de 3 ya filtra picos a nivel de muestra. Exigir varias
    // lecturas seguidas evita paradas espurias, que es lo que acaba llevando a
    // puentear una proteccion. El retraso de 2-3 s no cuenta en una planta con
    // constante de tiempo de decenas de minutos.
    if (problema)              fallosSeguidos++; else fallosSeguidos = 0;
    // Se vigilan las dos: asi un offset mal ajustado nunca puede subir el
    // punto de corte efectivo, solo bajarlo.
    float tVigilada = fmaxf(t, g_tCruda);
    if (tVigilada > T_LIMITE_ABSOLUTO) sobreTempSegs++;  else sobreTempSegs  = 0;

    const bool energizado = (g_salidaArmada || g_estado == EST_ENSAYO);

    if (g_estado != EST_FALLA) {
      // Sobretemperatura: latcha SIEMPRE. Si la T supera el limite con la
      // salida en reposo, significa que el SSR conduce sin orden.
      if (sobreTempSegs >= DEBOUNCE_SOBRETEMP) {
        char b[48];
        snprintf(b, sizeof(b), "T=%.1f C supera limite %.0f C", tVigilada, T_LIMITE_ABSOLUTO);
        entrarEnFallaCritica(b);
      }
      // Sensor invalido: latcha solo si hay algo energizado. En reposo basta
      // con marcarlo invalido, que ya impide armar y lanzar el ensayo.
      else if (fallosSeguidos >= DEBOUNCE_SENSOR) {
        if (energizado) entrarEnFallaCritica(g_avisoSensor);
        else if (fallosSeguidos == DEBOUNCE_SENSOR)
          Serial.printf("# [!] Sensor no valido: %s. No se puede armar.\n", g_avisoSensor);
      }
      // Marcha en seco (provisional, ver §10.4)
      else if (!problema) {
        uint32_t ahora = millis();
        // §10.4: en la meseta de ebullicion el estancamiento es NORMAL, el agua
        // absorbe calor latente sin subir de temperatura. Vigilar ahi daria
        // falso positivo garantizado durante la purga. La firma real del seco
        // es la contraria: cuando el agua se acaba, la temperatura SUBE rapido,
        // y entonces ya estamos fuera de esta banda y la vigilancia vuelve.
        bool enEbullicion = fabsf(t - puntoEbullicion()) < BANDA_EBULLICION;
        if (g_dutyComandado >= SECO_DUTY_MINIMO && g_salidaArmada && !enEbullicion) {
          if (isnan(tMarcaSeco)) { tMarcaSeco = t; marcaSeco = ahora; }
          else if ((uint32_t)(ahora - marcaSeco) >= SECO_VENTANA_MS) {
            if (t - tMarcaSeco < SECO_DELTA_MIN) {
              char b[48];
              snprintf(b, sizeof(b), "sin calentar: +%.1f C en %lu min al %d%%",
                       t - tMarcaSeco,
                       (unsigned long)(SECO_VENTANA_MS / 60000), g_dutyComandado);
              entrarEnFalla(b);
            } else { tMarcaSeco = t; marcaSeco = ahora; }
          }
        } else tMarcaSeco = NAN;
      }
    }

    // -- Ciclo de esterilizacion: la maquina de estados manda la salida --
    if (g_estado == EST_CICLO && g_sensorValido) {
      g_dutyComandado = avanzarCiclo(t);
      if (g_fase == F_TERMINADO || g_fase == F_LISTO_ABRIR) {
        // Salida cortada, pero se permanece en EST_CICLO para seguir vigilando
        // el enfriamiento. No puede calentar: la salida esta desarmada.
        if (g_salidaArmada) {
          g_salidaArmada = 0;
          g_dutyComandado = 0;
          ponerSSR(false);
          g_pedirFlush = true;
          Serial.printf("# CICLO COMPLETO. F0 alcanzado: %.2f min\n", g_F0);
          Serial.printf("# Enfriando. Aviso al bajar de %.0f C.\n", T_DESPRESURIZADO_C);
        }
      } else if ((uint32_t)(millis() - g_inicioEnsayo) >= g_duracionEnsayoMs) {
        entrarEnFalla("tiempo maximo agotado sin completar F0");
      }
    }

    // -- Lazo de control PI, ritmo fijo de 1 s (§6: 500 ms - 1 s) --
    if (g_estado == EST_PID && g_sensorValido) {
      if (g_enRampa) {
        // Fase de rampa: 100 % en lazo abierto. El PID no interviene, asi que
        // no hay integrador que cargar ni que desenganchar despues.
        if (t >= g_par.setpoint - ENTREGA_PID_C) {
          g_enRampa = false;
          reiniciarPID(g_par.setpoint);      // precarga justo en el relevo
          Serial.printf("# RELEVO a %.2f C: el PID toma el control\n", t);
          pitidoOk();
        } else {
          g_dutyComandado = 100;
        }
      } else {
        g_dutyComandado = (int)lroundf(calcularPI(g_par.setpoint, t, 1.0f));
      }
      if ((uint32_t)(millis() - g_inicioEnsayo) >= g_duracionEnsayoMs)
        terminarEnsayo("tiempo cumplido");
    }

    // -- Muestreo, tanto en escalon como en PID --
    if ((g_estado == EST_ENSAYO || g_estado == EST_PID || g_estado == EST_CICLO)
        && g_sensorValido) {
      if (g_nMuestras < MAX_MUESTRAS) g_muestras[g_nMuestras++] = (int16_t)lroundf(t * 100.0f);

      if (g_nMuestras >= MAX_MUESTRAS) terminarEnsayo("buffer lleno");
      else if (g_estado == EST_ENSAYO &&
               (uint32_t)(millis() - g_inicioEnsayo) >= g_duracionEnsayoMs)
        terminarEnsayo("tiempo cumplido");
      else if (g_nMuestras % MUESTRAS_FLUSH == 0) g_pedirFlush = true;
    }

    vTaskDelayUntil(&ultima, periodo);
  }
}

// ---------------------------------------------------------------------------
// ENCODER (PCNT, §6)
// ---------------------------------------------------------------------------
void iniciarEncoder() {
  pcnt_unit_config_t cfgU = {};
  cfgU.low_limit = -1000; cfgU.high_limit = 1000;
  ESP_ERROR_CHECK(pcnt_new_unit(&cfgU, &pcntUnidad));

  pcnt_glitch_filter_config_t cfgF = {};
  cfgF.max_glitch_ns = 1000;                     // antirrebote por hardware
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcntUnidad, &cfgF));

  pcnt_chan_config_t cA = {}; cA.edge_gpio_num = PIN_ENC_CLK; cA.level_gpio_num = PIN_ENC_DT;
  pcnt_channel_handle_t canA = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcntUnidad, &cA, &canA));
  pcnt_chan_config_t cB = {}; cB.edge_gpio_num = PIN_ENC_DT;  cB.level_gpio_num = PIN_ENC_CLK;
  pcnt_channel_handle_t canB = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcntUnidad, &cB, &canB));

  pcnt_channel_set_edge_action(canA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(canA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_channel_set_edge_action(canB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(canB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  ESP_ERROR_CHECK(pcnt_unit_enable(pcntUnidad));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcntUnidad));
  ESP_ERROR_CHECK(pcnt_unit_start(pcntUnidad));
  gpio_set_pull_mode(PIN_ENC_CLK, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(PIN_ENC_DT,  GPIO_PULLUP_ONLY);
}

int leerGiro() {
  static int resto = 0;
  int cuenta = 0;
  pcnt_unit_get_count(pcntUnidad, &cuenta);
  pcnt_unit_clear_count(pcntUnidad);
  cuenta += resto;
  int det = cuenta / PASOS_POR_DETENTE;
  resto = cuenta - det * PASOS_POR_DETENTE;
  return det;
}

int leerPulsador() {
  static bool pulsadoAntes = false;
  static uint32_t tPuls = 0;
  static bool largaEmitida = false;
  bool pulsado = (digitalRead(PIN_ENC_SW) == LOW);
  uint32_t ahora = millis();

  if (pulsado && !pulsadoAntes) { pulsadoAntes = true; tPuls = ahora; largaEmitida = false; }
  else if (pulsado && pulsadoAntes && !largaEmitida && (ahora - tPuls) >= PULSACION_LARGA_MS) {
    largaEmitida = true; return 2;
  } else if (!pulsado && pulsadoAntes) {
    pulsadoAntes = false;
    if (!largaEmitida) return 1;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// PANTALLA
// ---------------------------------------------------------------------------
bool respondeI2C(uint8_t dir) { Wire.beginTransmission(dir); return Wire.endTransmission() == 0; }

// Con alimentacion externa el rail sube mas despacio que por USB y el SSD1306
// puede no haber terminado su reset interno: por eso se sondea y se reintenta.
bool iniciarPantalla(uint8_t intentos) {
  Wire.begin(21, 22);
  Wire.setClock(VELOCIDAD_I2C);
  for (uint8_t i = 1; i <= intentos; i++) {
    for (uint8_t k = 0; k < sizeof(DIRECCIONES_OLED); k++) {
      uint8_t dir = DIRECCIONES_OLED[k];
      if (respondeI2C(dir)) {
        g_dirPantalla = dir;
        pantalla.setI2CAddress(dir << 1);
        pantalla.begin();
        pantalla.setBusClock(VELOCIDAD_I2C);
        pantalla.setContrast(200);
        g_pantallaOk = true;
        Serial.printf("# OLED en 0x%02X\n", dir);
        return true;
      }
    }
    delay(250);
  }
  g_pantallaOk = false;
  return false;
}

// ---------------------------------------------------------------------------
// INTERFAZ
// ---------------------------------------------------------------------------
enum Vista { V_PRINCIPAL, V_MENU, V_EDITAR, V_CONFIRMAR, V_CHECKLIST,
             V_ENSAYO, V_PID, V_CICLO, V_RESULTADOS, V_INFO, V_MANUAL, V_CALIBRACION };
Vista g_vista = V_PRINCIPAL;

enum TipoItem : uint8_t { IT_VALOR, IT_SUBMENU, IT_ACCION };
enum Accion : uint8_t { AC_MANUAL, AC_GUARDAR, AC_BUZZER, AC_VOLVER,
                        AC_INFO, AC_INICIAR_ENSAYO, AC_RESULTADOS, AC_INICIAR_PID,
                        AC_CALIBRAR };

struct ItemMenu {
  const char *etiqueta, *ayuda;
  uint8_t     tipo;
  float      *valor;
  float       minimo, maximo, paso;
  uint8_t     decimales;
  const char *unidad;
  uint8_t     arg;
};

float g_ventanaEditable = 2000;

// Menu del OPERADOR: SOLO lo esencial.
//
// Todo lo que puede alterar el proceso vive en la consola, que exige un PC y
// por tanto la presencia del tecnico. La razon no es desconfianza: es que
// varias de esas opciones no avisan cuando se tuercen.
//
//   MODO MANUAL y ENSAYO ESCALON aplican calor sin la gobernanza del ciclo.
//   KP/KI/KD y VENTANA PWM rompen el lazo de control.
//   CALIBRACION falsea la temperatura medida, y con ella el F0.
//   LETALIDAD F0 es el ajuste mas consecuente del equipo: bajarlo de 20 a 5
//   sub-esteriliza sin que nada lo delate.
//
// El operador SI puede verificar los parametros: se muestran en INFORMACION,
// en solo lectura. Ver sin poder cambiar.
const ItemMenu MENU_OPERADOR[] = {
  { "INICIAR CICLO", "ARRANCA LA ESTERILIZACION", IT_ACCION, NULL, 0,0,0,0, NULL, AC_INICIAR_PID },
  { "INFORMACION",   "VERSION Y PARAMETROS",      IT_ACCION, NULL, 0,0,0,0, NULL, AC_INFO },
  { "VOLVER",        "SALIR AL PANEL PRINCIPAL",  IT_ACCION, NULL, 0,0,0,0, NULL, AC_VOLVER },
};

struct Menu { const char *titulo; const ItemMenu *items; uint8_t n; int8_t padre; };
const Menu MENUS[] = {
  { "MENU", MENU_OPERADOR, sizeof(MENU_OPERADOR)/sizeof(ItemMenu), -1 },
};
const uint8_t N_MENUS = sizeof(MENUS)/sizeof(Menu);

uint8_t g_menuActual = 0;
int     g_cursor[N_MENUS] = {0};
int     g_scroll[N_MENUS] = {0};
const ItemMenu *g_itemEditando = NULL;

// -- Confirmacion y checklist --
// El cursor arranca SIEMPRE en NO: en un recipiente a presion el valor por
// defecto de cualquier pregunta tiene que ser el que no hace nada.
int     g_siNo = 0;
uint8_t g_puntoChecklist = 0;
bool    g_arrancandoPID = false;   // que se esta confirmando: escalon o PID

struct PuntoChecklist { const char *l1, *l2; };
const PuntoChecklist CHECKLIST[] = {
  { "VALVULA DE PURGA",     "ABIERTA"                },
  { "NIVEL DE AGUA ALTO,",  "RESISTENCIA CUBIERTA"   },
  { "VALVULA DE SEGURIDAD", "LIBRE Y SIN OBSTRUIR"   },
  { "TAPA BIEN CERRADA",    "Y ASEGURADA"            },
  { "SUSTRATO HUMEDO",      "SECO NO SE ESTERILIZA"  },
  { "VOY A ESTAR PRESENTE", "TODO EL CICLO"          },
};
const uint8_t N_CHECKLIST = sizeof(CHECKLIST)/sizeof(PuntoChecklist);

// --- Dibujo ---------------------------------------------------------------
void dibujarPrincipal() {
  char buf[24];
  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, g_salidaArmada ? "ACTIVA" : "EN REPOSO");
  const char *e = g_sensorValido ? "OK" : "---";
  pantalla.drawStr(128 - pantalla.getStrWidth(e), 8, e);
  pantalla.drawHLine(0, 11, 128);

  if (g_sensorValido) snprintf(buf, sizeof(buf), "%.1f", g_temperatura);
  else                snprintf(buf, sizeof(buf), "--.-");
  pantalla.setFont(u8g2_font_logisoso20_tn);
  int a = pantalla.getStrWidth(buf);
  pantalla.drawStr(64 - (a + 14)/2, 38, buf);
  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(64 + (a + 14)/2 - 12, 30, "C");

  snprintf(buf, sizeof(buf), "META %.1f", g_par.setpoint);
  pantalla.drawStr(0, 50, buf);
  snprintf(buf, sizeof(buf), "%3d%%", g_dutyComandado);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf), 50, buf);

  pantalla.drawFrame(0, 54, 128, 9);
  int w = (g_dutyComandado * 126)/100;
  if (w > 0) pantalla.drawBox(1, 55, w, 7);

  // Sensor con problema pero sin latchar: avisar sin bloquear el equipo
  if (!g_sensorValido && g_avisoSensor[0]) {
    pantalla.setDrawColor(0); pantalla.drawBox(0, 40, 128, 11);
    pantalla.setDrawColor(1);
    pantalla.setFont(u8g2_font_5x8_tf);
    pantalla.drawStr(0, 49, g_avisoSensor);
  }
}

void dibujarFalla() {
  pantalla.setFont(u8g2_font_7x13_tf);
  pantalla.drawBox(0, 0, 128, 15);
  pantalla.setDrawColor(0);
  pantalla.drawStr(3, 12, g_fallaCritica ? "** FALLA GRAVE **" : "*** FALLA ***");
  pantalla.setDrawColor(1);

  // El motivo puede no caber en una linea. Se parte por un ESPACIO, no a los
  // 25 caracteres exactos: cortar "LIMITE 128 C" como "LIMITE 1" / "28 C"
  // obliga al operador a reconstruir mentalmente lo que lee.
  pantalla.setFont(u8g2_font_5x8_tf);
  char l1[27] = "", l2[27] = "";
  size_t n = strlen(g_motivoFalla);
  if (n <= 25) {
    strncpy(l1, g_motivoFalla, 26);
  } else {
    int corte = 25;
    while (corte > 10 && g_motivoFalla[corte] != ' ') corte--;
    if (corte <= 10) corte = 25;          // sin espacio util, se corta igual
    strncpy(l1, g_motivoFalla, corte);
    l1[corte] = '\0';
    strncpy(l2, g_motivoFalla + corte + 1, 25);
  }
  pantalla.drawStr(0, 28, l1);
  pantalla.drawStr(0, 38, l2);

  pantalla.drawHLine(0, 44, 128);
  pantalla.drawStr(0, 54, "SALIDA CORTADA");
  if (g_fallaCritica) pantalla.drawStr(0, 62, "REQUIERE REVISION TECNICA");
  else                pantalla.drawStr(0, 62, "DEJE PULSADO PARA SALIR");
}

void dibujarMenu() {
  const Menu &m = MENUS[g_menuActual];
  int &cur = g_cursor[g_menuActual];
  int &scr = g_scroll[g_menuActual];
  const uint8_t VIS = 3;
  char buf[32];

  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, m.titulo);
  snprintf(buf, sizeof(buf), "%d/%d", cur + 1, m.n);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf), 8, buf);
  pantalla.drawHLine(0, 11, 128);

  if (cur < scr)        scr = cur;
  if (cur >= scr + VIS) scr = cur - VIS + 1;

  pantalla.setFont(u8g2_font_7x13_tf);
  for (uint8_t i = 0; i < VIS; i++) {
    int idx = scr + i;
    if (idx >= m.n) break;
    int y = 24 + i * 13;
    const ItemMenu &it = m.items[idx];
    if (idx == cur) { pantalla.drawBox(0, y - 11, 128, 13); pantalla.setDrawColor(0); }
    pantalla.drawStr(3, y, it.etiqueta);
    if (it.tipo == IT_VALOR) {
      if (it.unidad) snprintf(buf, sizeof(buf), "%.*f%s", it.decimales, *it.valor, it.unidad);
      else           snprintf(buf, sizeof(buf), "%.*f",   it.decimales, *it.valor);
      pantalla.drawStr(125 - pantalla.getStrWidth(buf), y, buf);
    } else if (it.tipo == IT_SUBMENU) pantalla.drawStr(117, y, ">");
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
  int w = pantalla.getStrWidth(buf), wu = 0;
  if (it.unidad) { pantalla.setFont(u8g2_font_7x13_tf); wu = pantalla.getStrWidth(it.unidad) + 4; }
  int x = 64 - (w + wu)/2;
  pantalla.setFont(u8g2_font_logisoso20_tn);
  pantalla.drawStr(x, 40, buf);
  if (it.unidad) { pantalla.setFont(u8g2_font_7x13_tf); pantalla.drawStr(x + w + 4, 40, it.unidad); }

  pantalla.setFont(u8g2_font_5x8_tf);
  snprintf(buf, sizeof(buf), "min %.*f   max %.*f", it.decimales, it.minimo, it.decimales, it.maximo);
  pantalla.drawStr(64 - pantalla.getStrWidth(buf)/2, 52, buf);
  const char *p = "girar cambia | clic OK";
  pantalla.drawStr(64 - pantalla.getStrWidth(p)/2, 62, p);
}

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
  char buf[32];
  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, "CONFIRMAR");
  pantalla.drawHLine(0, 11, 128);
  if (g_arrancandoPID) {
    // Se confirman los DOS parametros del ciclo, no solo la temperatura: el
    // F0 objetivo es el que decide cuando termina, asi que debe verse aqui.
    pantalla.drawStr(0, 24, "INICIAR CICLO A");
    snprintf(buf, sizeof(buf), "%.0f C  /  F0 %.0f MIN",
             g_par.setpoint, g_par.f0Objetivo);
    pantalla.drawStr(0, 35, buf);
  } else {
    snprintf(buf, sizeof(buf), "Ensayo %d%% / %d min",
             (int)g_par.dutyEnsayo, (int)g_par.minutosEnsayo);
    pantalla.drawStr(0, 24, buf);
    pantalla.drawStr(0, 35, "ENERGIZA LA RESISTENCIA");
  }
  dibujarSiNo(51);
  pantalla.setFont(u8g2_font_5x8_tf);
  const char *p = "GIRAR ELIGE | CLIC ACEPTA";
  pantalla.drawStr(64 - pantalla.getStrWidth(p)/2, 63, p);
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
  const char *t = "NO CANCELA EL CICLO";
  pantalla.drawStr(64 - pantalla.getStrWidth(t)/2, 63, t);
}

// Ensayo en curso: temperatura, tiempo y la curva registrada hasta ahora
void dibujarEnsayo() {
  char buf[32];
  uint32_t tr = (millis() - g_inicioEnsayo)/1000;
  uint32_t tt = g_duracionEnsayoMs/1000;

  pantalla.setFont(u8g2_font_6x10_tf);
  snprintf(buf, sizeof(buf), "ENSAYO %d%%", g_dutyEnsayo);
  pantalla.drawStr(0, 8, buf);
  snprintf(buf, sizeof(buf), "%lu/%lu min",
           (unsigned long)tr/60, (unsigned long)tt/60);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf), 8, buf);
  pantalla.drawHLine(0, 11, 128);

  snprintf(buf, sizeof(buf), "%.1f", g_temperatura);
  pantalla.setFont(u8g2_font_logisoso20_tn);
  pantalla.drawStr(0, 33, buf);
  pantalla.setFont(u8g2_font_5x8_tf);
  if (!isnan(g_tInicioEnsayo)) {
    snprintf(buf, sizeof(buf), "%+.1f C", g_temperatura - g_tInicioEnsayo);
    pantalla.drawStr(128 - pantalla.getStrWidth(buf), 22, buf);
  }
  snprintf(buf, sizeof(buf), "%u muestras", g_nMuestras);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf), 32, buf);

  // Grafica de la curva completa, reescalada a 128x24 px
  const int Y0 = 38, ALTO = 24;
  pantalla.drawFrame(0, Y0, 128, ALTO);
  uint16_t n = g_nMuestras;
  if (n >= 2) {
    int16_t mn = g_muestras[0], mx = g_muestras[0];
    for (uint16_t i = 1; i < n; i++) {
      if (g_muestras[i] < mn) mn = g_muestras[i];
      if (g_muestras[i] > mx) mx = g_muestras[i];
    }
    int rango = mx - mn;
    if (rango < 100) rango = 100;              // escala minima de 1 C
    int prevY = -1;
    for (int px = 0; px < 126; px++) {
      uint16_t idx = (uint32_t)px * (n - 1) / 125;
      int y = Y0 + ALTO - 2 - ((int32_t)(g_muestras[idx] - mn) * (ALTO - 3)) / rango;
      if (prevY >= 0) pantalla.drawLine(px, prevY, px + 1, y);
      prevY = y;
    }
  }
}

// Lazo PI en curso: lo que importa aqui es ver si converge y si se pasa.
// Por eso arriba van consigna y medida juntas, y abajo el error con signo.
void dibujarPID() {
  char buf[32];
  uint32_t tr = (millis() - g_inicioEnsayo) / 1000;
  uint32_t tt = g_duracionEnsayoMs / 1000;

  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, g_enRampa ? "RAMPA 100%" : "CONTROL PID");
  snprintf(buf, sizeof(buf), "%lu/%lu min", (unsigned long)tr / 60,
           (unsigned long)tt / 60);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf), 8, buf);
  pantalla.drawHLine(0, 11, 128);

  snprintf(buf, sizeof(buf), "%.1f", g_temperatura);
  pantalla.setFont(u8g2_font_logisoso20_tn);
  pantalla.drawStr(0, 33, buf);

  pantalla.setFont(u8g2_font_5x8_tf);
  snprintf(buf, sizeof(buf), "SP %.1f C", g_par.setpoint);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf), 22, buf);
  snprintf(buf, sizeof(buf), "err %+.2f", g_errorPID);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf), 32, buf);

  // Barra de salida con la cifra dentro
  pantalla.drawFrame(0, 38, 128, 10);
  int w = (g_dutyComandado * 126) / 100;
  if (w > 0) pantalla.drawBox(1, 39, w, 8);
  snprintf(buf, sizeof(buf), "%d%%", g_dutyComandado);
  pantalla.setDrawColor(2);                     // XOR: legible sobre la barra
  pantalla.drawStr(54, 46, buf);
  pantalla.setDrawColor(1);

  // Curva registrada, para ver la aproximacion al setpoint
  const int Y0 = 50, ALTO = 14;
  uint16_t n = g_nMuestras;
  if (n >= 2) {
    int16_t mn = g_muestras[0], mx = g_muestras[0];
    for (uint16_t i = 1; i < n; i++) {
      if (g_muestras[i] < mn) mn = g_muestras[i];
      if (g_muestras[i] > mx) mx = g_muestras[i];
    }
    int rango = mx - mn;
    if (rango < 100) rango = 100;
    int prevY = -1;
    for (int px = 0; px < 128; px++) {
      uint16_t idx = (uint32_t)px * (n - 1) / 127;
      int y = Y0 + ALTO - 1 - ((int32_t)(g_muestras[idx] - mn) * (ALTO - 2)) / rango;
      if (prevY >= 0) pantalla.drawLine(px, prevY, px + 1, y);
      prevY = y;
    }
  }
}

// Pantalla de calibracion contra patron de laboratorio (§10.1).
//
// Pensada para el gesto real del operador: mirar el termometro patron, mirar
// la pantalla, y girar hasta que el valor CORREGIDO coincida. Por eso se
// muestran a la vez la lectura sin corregir, el offset y el resultado: asi se
// ve lo que se esta haciendo en lugar de calcular a ciegas.
//
// Se calibra a temperatura de trabajo con el equipo montado, no en un baño de
// hielo con la sonda suelta: lo que interesa es el error del conjunto
// sonda + termowell + montaje en el punto donde importa.
void dibujarCalibracion() {
  char buf[32];

  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, "CALIBRACION");
  pantalla.drawHLine(0, 11, 128);

  pantalla.setFont(u8g2_font_5x8_tf);
  pantalla.drawStr(0, 21, "SIN CORREGIR");
  snprintf(buf, sizeof(buf), "%.2f C", g_tCruda);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf), 21, buf);

  pantalla.drawStr(0, 31, "OFFSET");
  snprintf(buf, sizeof(buf), "%+.2f C", g_par.offsetC);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf), 31, buf);

  pantalla.drawHLine(0, 35, 128);

  // El valor corregido, grande: es el que debe igualar al patron
  pantalla.setFont(u8g2_font_5x8_tf);
  pantalla.drawStr(0, 45, "IGUALAR AL PATRON:");
  snprintf(buf, sizeof(buf), "%.2f", g_temperatura);
  pantalla.setFont(u8g2_font_logisoso20_tn);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf) - 16, 62, buf);
  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(114, 54, "C");

  pantalla.setFont(u8g2_font_5x8_tf);
  pantalla.drawStr(0, 62, "GIRAR");
  pantalla.drawStr(0, 54, "CLIC=OK");
}

// Pantalla del ciclo. Cada fase muestra lo que el operador necesita en ESE
// momento, no un panel generico.
//
// La cabecera lleva el cronometro del ciclo completo, que se congela al
// terminar la esterilizacion: lo que interesa registrar es cuanto duro el
// proceso, no cuanto lleva enfriando. La temperatura no va en la cabecera
// porque cada fase ya la muestra donde le corresponde.
void dibujarCiclo() {
  char buf[32];
  uint32_t enFase = (millis() - g_inicioFase) / 1000;
  uint32_t total  = segundosCiclo();

  // --- Cabecera: fase + tiempo total del ciclo ---
  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, nombreFase(g_fase));
  snprintf(buf, sizeof(buf), "%lu:%02lu", total / 60, total % 60);
  pantalla.drawStr(128 - pantalla.getStrWidth(buf), 8, buf);
  pantalla.drawHLine(0, 11, 128);

  switch (g_fase) {

    case F_CALENTANDO: {
      pantalla.setFont(u8g2_font_logisoso20_tn);
      snprintf(buf, sizeof(buf), "%.1f", g_temperatura);
      pantalla.drawStr(2, 36, buf);
      pantalla.setFont(u8g2_font_5x8_tf);
      snprintf(buf, sizeof(buf), "HIERVE A %.1f", puntoEbullicion());
      pantalla.drawStr(128 - pantalla.getStrWidth(buf), 24, buf);
      pantalla.drawStr(128 - pantalla.getStrWidth("100%"), 34, "100%");
      pantalla.drawHLine(0, 42, 128);
      pantalla.setFont(u8g2_font_6x10_tf);
      pantalla.drawStr(0, 53, "VALVULA ABIERTA");
      pantalla.setFont(u8g2_font_5x8_tf);
      pantalla.drawStr(0, 63, "ESPERE A LA EBULLICION");
      break;
    }

    case F_PURGA: {
      uint32_t tot = PURGA_MS / 1000;
      uint32_t resta = (enFase < tot) ? tot - enFase : 0;

      // Cuenta atras grande a la izquierda, temperatura a la derecha
      pantalla.setFont(u8g2_font_logisoso20_tn);
      snprintf(buf, sizeof(buf), "%lu:%02lu", resta / 60, resta % 60);
      pantalla.drawStr(2, 33, buf);
      pantalla.setFont(u8g2_font_6x10_tf);
      snprintf(buf, sizeof(buf), "%.1fC", g_temperatura);
      pantalla.drawStr(128 - pantalla.getStrWidth(buf), 31, buf);

      pantalla.drawFrame(0, 37, 128, 8);
      int w = (int)((enFase * 126) / tot);
      if (w > 126) w = 126;
      if (w > 0) pantalla.drawBox(1, 38, w, 6);

      // Dos lineas con 9 px de separacion: con la fuente 5x8 no se solapan
      pantalla.setFont(u8g2_font_5x8_tf);
      pantalla.drawStr(0, 54, "DEBE SALIR VAPOR CONTINUO");
      pantalla.drawStr(0, 63, "POR LA VALVULA ABIERTA");
      break;
    }

    case F_CERRAR_VALVULA: {
      bool parpadeo = ((millis() / 500) % 2) == 0;
      if (parpadeo) { pantalla.drawBox(0, 13, 128, 17); pantalla.setDrawColor(0); }
      pantalla.setFont(u8g2_font_7x13_tf);
      pantalla.drawStr(6, 26, "CIERRE LA VALVULA");
      pantalla.setDrawColor(1);

      pantalla.setFont(u8g2_font_5x8_tf);
      snprintf(buf, sizeof(buf), "%.1f C   PURGA LISTA", g_temperatura);
      pantalla.drawStr(0, 41, buf);
      if (g_avisosCierre > 0) {
        snprintf(buf, sizeof(buf), "LA TEMPERATURA NO SUBE %u", g_avisosCierre);
        pantalla.drawStr(0, 52, buf);
        pantalla.drawStr(0, 62, "SIGUE ABIERTA LA VALVULA?");
      } else {
        pantalla.drawStr(0, 54, "CIERRE LA VALVULA AHORA");
      }
      break;
    }

    case F_RAMPA: {
      pantalla.setFont(u8g2_font_logisoso20_tn);
      snprintf(buf, sizeof(buf), "%.1f", g_temperatura);
      pantalla.drawStr(2, 36, buf);
      pantalla.setFont(u8g2_font_5x8_tf);
      snprintf(buf, sizeof(buf), "META %.1f", g_par.setpoint);
      pantalla.drawStr(128 - pantalla.getStrWidth(buf), 24, buf);
      pantalla.drawStr(128 - pantalla.getStrWidth("100%"), 34, "100%");
      pantalla.drawHLine(0, 42, 128);
      pantalla.setFont(u8g2_font_6x10_tf);
      pantalla.drawStr(0, 53, "VALVULA CERRADA OK");
      pantalla.setFont(u8g2_font_5x8_tf);
      snprintf(buf, sizeof(buf), "SUBIENDO A %.0f C", g_par.setpoint);
      pantalla.drawStr(0, 63, buf);
      break;
    }

    case F_MESETA: {
      pantalla.setFont(u8g2_font_logisoso20_tn);
      snprintf(buf, sizeof(buf), "%.1f", g_temperatura);
      pantalla.drawStr(2, 34, buf);

      pantalla.setFont(u8g2_font_5x8_tf);
      snprintf(buf, sizeof(buf), "META %.1f", g_par.setpoint);
      pantalla.drawStr(128 - pantalla.getStrWidth(buf), 22, buf);
      snprintf(buf, sizeof(buf), "%d%%", g_dutyComandado);
      pantalla.drawStr(128 - pantalla.getStrWidth(buf), 32, buf);

      // F0 es el dato que decide cuando termina: va destacado
      pantalla.setFont(u8g2_font_6x10_tf);
      snprintf(buf, sizeof(buf), "F0 %.1f / %.0f", g_F0, g_par.f0Objetivo);
      pantalla.drawStr(0, 46, buf);

      pantalla.drawFrame(0, 50, 128, 9);
      int w = (int)((g_F0 * 126.0f) / g_par.f0Objetivo);
      if (w > 126) w = 126;
      if (w > 0) pantalla.drawBox(1, 51, w, 7);

      pantalla.setFont(u8g2_font_5x8_tf);
      float tasa = powf(10.0f, (g_temperatura - 121.1f) / 10.0f);
      snprintf(buf, sizeof(buf), "SUMA %.2f F0 POR MINUTO", tasa);
      pantalla.drawStr(0, 63, buf);
      break;
    }

    case F_TERMINADO: {
      pantalla.setFont(u8g2_font_7x13_tf);
      pantalla.drawStr(2, 25, "CICLO COMPLETO");
      pantalla.setFont(u8g2_font_6x10_tf);
      snprintf(buf, sizeof(buf), "F0 %.1f  en %lu:%02lu", g_F0, total / 60, total % 60);
      pantalla.drawStr(0, 37, buf);
      pantalla.drawHLine(0, 41, 128);

      pantalla.setFont(u8g2_font_5x8_tf);
      pantalla.drawStr(0, 50, "NO ABRIR: AUN HAY PRESION");
      snprintf(buf, sizeof(buf), "%.1f C  ->  aviso a %.0f C",
               g_temperatura, T_DESPRESURIZADO_C);
      pantalla.drawStr(0, 59, buf);

      int w = (int)(126.0f * (g_par.setpoint - g_temperatura) /
                    (g_par.setpoint - T_DESPRESURIZADO_C));
      if (w < 0) w = 0;
      if (w > 126) w = 126;
      pantalla.drawFrame(0, 60, 128, 4);
      if (w > 0) pantalla.drawBox(1, 61, w, 2);
      break;
    }

    case F_LISTO_ABRIR: {
      bool parpadeo = ((millis() / 600) % 2) == 0;
      if (parpadeo) { pantalla.drawBox(0, 13, 128, 16); pantalla.setDrawColor(0); }
      pantalla.setFont(u8g2_font_7x13_tf);
      pantalla.drawStr(4, 25, "DESPRESURIZADO");
      pantalla.setDrawColor(1);

      pantalla.setFont(u8g2_font_6x10_tf);
      snprintf(buf, sizeof(buf), "%.1f C   F0 %.1f", g_temperatura, g_F0);
      pantalla.drawStr(0, 38, buf);
      pantalla.drawHLine(0, 42, 128);

      // El aviso NO autoriza a abrir: la temperatura es una inferencia, el
      // manometro es la medida directa y sigue siendo la autoridad.
      pantalla.setFont(u8g2_font_5x8_tf);
      pantalla.drawStr(0, 52, "VERIFIQUE EL MANOMETRO");
      pantalla.drawStr(0, 62, "EN CERO ANTES DE ABRIR");
      break;
    }
  }
}

void dibujarResultados() {
  char buf[32];
  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, "RESULTADOS");
  pantalla.drawHLine(0, 11, 128);

  if (!g_res.valido) {
    pantalla.setFont(u8g2_font_7x13_tf);
    pantalla.drawStr(0, 30, "SIN ENSAYO");
    pantalla.drawStr(0, 44, "VALIDO AUN");
    return;
  }
  pantalla.setFont(u8g2_font_7x13_tf);
  snprintf(buf, sizeof(buf), "L   %.0f s",  g_res.L);   pantalla.drawStr(2, 26, buf);
  snprintf(buf, sizeof(buf), "tau %.0f s",  g_res.tau); pantalla.drawStr(2, 39, buf);
  snprintf(buf, sizeof(buf), "K   %.3f",    g_res.K);   pantalla.drawStr(2, 52, buf);

  pantalla.setFont(u8g2_font_5x8_tf);
  if (g_res.L > 0 && g_res.K > 0) {
    float kp = 0.9f * g_res.tau / (g_res.K * g_res.L);
    snprintf(buf, sizeof(buf), "Kp %.1f  Ki %.4f", kp, kp / (3.33f * g_res.L));
    pantalla.drawStr(0, 62, buf);
  }
}

void dibujarInfo() {
  char buf[32];
  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, "INFORMACION");
  pantalla.drawHLine(0, 11, 128);
  pantalla.setFont(u8g2_font_7x13_tf);
  pantalla.drawStr(2, 23, "PID AUTOCLAVE 75X");
  pantalla.setFont(u8g2_font_5x8_tf);
  snprintf(buf, sizeof(buf), "FIRMWARE %s", VERSION_FW);
  pantalla.drawStr(2, 33, buf);
  // Parametros del ciclo en SOLO LECTURA: el operador puede verificar que el
  // equipo esta configurado como debe antes de cargar, sin poder cambiarlo.
  snprintf(buf, sizeof(buf), "CICLO %.1f C / F0 %.0f MIN",
           g_par.setpoint, g_par.f0Objetivo);
  pantalla.drawStr(2, 42, buf);
  // La altitud y el punto de ebullicion no son adorno: son lo que distingue a
  // este equipo de uno calibrado a nivel del mar, y explican por que hacen
  // falta 17.5 psi para 121 C en lugar de los 15 que dice el manometro.
  snprintf(buf, sizeof(buf), "%.0f m - HIERVE A %.1f C", ALTITUD_M, puntoEbullicion());
  pantalla.drawStr(2, 50, buf);
  pantalla.drawHLine(0, 53, 128);
  pantalla.setFont(u8g2_font_7x13_tf);
  pantalla.drawStr(128 - pantalla.getStrWidth(AUTOR_FW) - 3, 64, AUTOR_FW);
}

void dibujarManual() {
  char buf[32];
  pantalla.setFont(u8g2_font_6x10_tf);
  pantalla.drawStr(0, 8, "MODO MANUAL");
  pantalla.drawHLine(0, 11, 128);
  if (!g_salidaArmada) {
    pantalla.setFont(u8g2_font_7x13_tf);
    pantalla.drawStr(0, 27, "SALIDA");
    pantalla.drawStr(0, 41, "EN REPOSO");
    pantalla.setFont(u8g2_font_5x8_tf);
    pantalla.drawStr(0, 52, "CONSOLA: ARMAR");
    pantalla.drawStr(0, 62, "PULSACION LARGA = SALIR");
    return;
  }
  snprintf(buf, sizeof(buf), "%d", g_dutyComandado);
  pantalla.setFont(u8g2_font_logisoso20_tn);
  pantalla.drawStr(50 - pantalla.getStrWidth(buf)/2, 38, buf);
  pantalla.setFont(u8g2_font_7x13_tf);
  pantalla.drawStr(74, 38, "% SALIDA");
  pantalla.drawFrame(0, 44, 128, 9);
  int w = (g_dutyComandado * 126)/100;
  if (w > 0) pantalla.drawBox(1, 45, w, 7);
  pantalla.setFont(u8g2_font_5x8_tf);
  snprintf(buf, sizeof(buf), "real %.1f%%   larga = salir", g_dutyReal);
  pantalla.drawStr(0, 62, buf);
}

void refrescarPantalla() {
  if (!g_pantallaOk) return;
  pantalla.clearBuffer();
  if (g_estado == EST_FALLA) dibujarFalla();      // la falla manda sobre todo
  else switch (g_vista) {
    case V_PRINCIPAL:  dibujarPrincipal();  break;
    case V_MENU:       dibujarMenu();       break;
    case V_EDITAR:     dibujarEditar();     break;
    case V_CONFIRMAR:  dibujarConfirmar();  break;
    case V_CHECKLIST:  dibujarChecklist();  break;
    case V_ENSAYO:     dibujarEnsayo();     break;
    case V_PID:        dibujarPID();        break;
    case V_CICLO:      dibujarCiclo();      break;
    case V_CALIBRACION: dibujarCalibracion(); break;
    case V_RESULTADOS: dibujarResultados(); break;
    case V_INFO:       dibujarInfo();       break;
    case V_MANUAL:     dibujarManual();     break;
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

void subirNivel() {
  if (MENUS[g_menuActual].padre >= 0) g_menuActual = MENUS[g_menuActual].padre;
  else                                g_vista = V_PRINCIPAL;
}

void cancelarArranque(const char *motivo) {
  g_puntoChecklist = 0;
  g_siNo  = 0;
  g_vista = V_MENU;
  Serial.printf("# Ensayo cancelado: %s\n", motivo);
}

void lanzarEnsayo() {
  g_dutyEnsayo       = (int)g_par.dutyEnsayo;
  g_duracionEnsayoMs = (uint32_t)g_par.minutosEnsayo * 60000UL;
  g_nMuestras        = 0;
  g_tInicioEnsayo    = g_temperatura;
  g_inicioEnsayo     = millis();
  g_salidaArmada     = 1;
  g_dutyComandado    = g_dutyEnsayo;
  g_estado           = EST_ENSAYO;
  g_vista            = V_ENSAYO;
  Serial.printf("# ENSAYO INICIADO: %d %% durante %d min. T0=%.2f C\n",
                g_dutyEnsayo, (int)g_par.minutosEnsayo, g_tInicioEnsayo);
  pitidoAviso();
}

void lanzarPID() {
  g_duracionEnsayoMs = (uint32_t)g_par.minutosEnsayo * 60000UL;
  g_nMuestras     = 0;
  g_dutyEnsayo    = 0;                 // en PID la salida la manda el lazo
  g_tInicioEnsayo = g_temperatura;
  g_inicioEnsayo  = millis();
  // Si falta mucho para el setpoint se sube en lazo abierto y el PID releva
  // cerca; si ya estamos cerca, el PID entra directo con su precarga.
  g_enRampa = (g_temperatura < g_par.setpoint - ENTREGA_PID_C);
  if (g_enRampa)
    Serial.printf("# RAMPA al 100 %% hasta %.1f C, luego releva el PID\n",
                  g_par.setpoint - ENTREGA_PID_C);
  else           reiniciarPID(g_par.setpoint);
  g_salidaArmada  = 1;
  g_dutyComandado = 0;
  g_estado        = EST_PID;
  g_vista         = V_PID;
  Serial.printf("# PID INICIADO: SP=%.1f C durante %d min. "
                "Kp=%.2f Ki=%.4f. T0=%.2f C\n",
                g_par.setpoint, (int)g_par.minutosEnsayo,
                g_par.kp, g_par.ki, g_tInicioEnsayo);
  pitidoAviso();
}

void lanzarCiclo() {
  if (g_estado == EST_FALLA) {     // red de seguridad: no deberia llegar aqui
    Serial.println(F("# Hay una FALLA activa. No se lanza el ciclo."));
    return;
  }
  g_duracionEnsayoMs = (uint32_t)g_par.minutosEnsayo * 60000UL;
  g_nMuestras     = 0;
  g_dutyEnsayo    = 0;
  g_tInicioEnsayo = g_temperatura;
  g_inicioEnsayo  = millis();
  g_F0            = 0.0f;
  g_finCiclo      = 0;
  g_tAlCerrar     = NAN;
  g_ultimoAviso   = 0;
  g_avisosCierre  = 0;
  reiniciarPID(g_par.setpoint);
  g_fase          = F_CALENTANDO;
  g_inicioFase    = millis();
  g_salidaArmada  = 1;
  g_dutyComandado = 100;
  g_estado        = EST_CICLO;
  g_vista         = V_CICLO;
  Serial.printf("# CICLO INICIADO: %.1f C, F0 objetivo %.0f min. T0=%.2f C\n",
                g_par.setpoint, g_par.f0Objetivo, g_tInicioEnsayo);
  Serial.println(F("# Fase 1: calentando con la valvula de purga ABIERTA"));
  pitidoAviso();
}

void aplicarGiro(int giro) {
  if (g_estado == EST_FALLA) return;
  switch (g_vista) {
    case V_PRINCIPAL: case V_INFO: case V_RESULTADOS: case V_ENSAYO: case V_PID:
    case V_CICLO:
      break;

    case V_CONFIRMAR: case V_CHECKLIST: {
      int nuevo = (giro > 0) ? 1 : 0;            // horario = SI
      if (nuevo != g_siNo) { g_siNo = nuevo; pitidoClic(); }
      break;
    }

    case V_MENU: {
      const Menu &m = MENUS[g_menuActual];
      int &cur = g_cursor[g_menuActual];
      cur += giro;
      if (cur < 0)    cur = 0;
      if (cur >= m.n) cur = m.n - 1;
      pitidoClic();
      break;
    }

    case V_EDITAR: {
      const ItemMenu &it = *g_itemEditando;
      float v = *it.valor + giro * it.paso;
      if (v < it.minimo) v = it.minimo;
      if (v > it.maximo) v = it.maximo;
      *it.valor = v;
      pitidoClic();
      break;
    }

    case V_CALIBRACION: {
      float v = g_par.offsetC + giro * 0.05f;
      if (v >  OFFSET_MAX_C) v =  OFFSET_MAX_C;
      if (v < -OFFSET_MAX_C) v = -OFFSET_MAX_C;
      g_par.offsetC = v;
      pitidoClic();
      break;
    }

    case V_MANUAL: {
      if (!g_salidaArmada) break;
      int d = g_dutyComandado + giro;
      g_dutyComandado = (d < 0) ? 0 : (d > 100 ? 100 : d);
      pitidoClic();
      break;
    }
  }
}

void aplicarPulsacion(int tipo) {
  if (g_estado == EST_FALLA) {
    // §7.2 exige que la FALLA sea terminal y que solo se salga por accion
    // deliberada del operador. Una pulsacion LARGA lo es. Sin esto, sin PC
    // delante habria que cortar la alimentacion, que es peor: se pierde el
    // contexto y el operador puede no saber que debe hacerlo.
    // Reconocer NO es rearmar: la salida sigue desarmada al salir.
    if (tipo == 2 && !g_fallaCritica) {
      g_estado = EST_IDLE;
      g_motivoFalla[0] = '\0';
      g_vista = V_PRINCIPAL;
      Serial.println(F("# FALLA reconocida desde el encoder. Salida sigue desarmada."));
      pitidoOk();
    } else if (tipo == 2) {
      avisoAccion();        // suena pero no cede: esta es de las graves
      Serial.println(F("# FALLA CRITICA: no se reconoce desde el panel."));
    }
    return;
  }

  if (tipo == 2) {                               // larga = atras / abortar
    switch (g_vista) {
      case V_ENSAYO:
        terminarEnsayo("abortado por el operador");
        g_vista = V_RESULTADOS;
        break;
      case V_PID:
        terminarEnsayo("PID detenido por el operador");
        g_vista = V_MENU;
        break;
      case V_CICLO:
        // Si el ciclo ya termino, la pulsacion larga solo cierra el aviso.
        if (g_estado == EST_CICLO &&
            (g_fase == F_TERMINADO || g_fase == F_LISTO_ABRIR)) {
          g_estado = EST_IDLE;
          g_vista  = V_PRINCIPAL;
          pitidoOk();
          Serial.println(F("# Aviso de despresurizacion cerrado por el operador."));
          break;
        }
        if (g_estado == EST_CICLO) {
          desarmar("ciclo abortado por el operador");
          g_estado = EST_IDLE;
          g_pedirFlush = true;
          Serial.printf("# CICLO ABORTADO en fase %s. F0 = %.2f min\n",
                        nombreFase(g_fase), g_F0);
        }
        g_vista = V_MENU;
        break;
      case V_MANUAL:    g_dutyComandado = 0; g_vista = V_MENU; break;
      case V_EDITAR:    confirmarEdicion();  g_vista = V_MENU; break;
      case V_INFO: case V_RESULTADOS: g_vista = V_MENU; break;
      case V_CALIBRACION:   // larga = salir sin guardar en NVS
        cargarParametros(); g_vista = V_MENU; break;
      case V_CONFIRMAR: case V_CHECKLIST: cancelarArranque("operador"); break;
      case V_MENU:      subirNivel(); break;
      default:          g_vista = V_PRINCIPAL; break;
    }
    pitidoOk();
    return;
  }
  if (tipo != 1) return;

  switch (g_vista) {
    case V_PRINCIPAL: g_menuActual = 0; g_vista = V_MENU; pitidoOk(); break;
    case V_INFO: case V_RESULTADOS: g_vista = V_MENU; pitidoOk(); break;
    case V_CALIBRACION:
      guardarParametros();          // clic corto = aceptar y grabar
      g_vista = V_MENU; pitidoOk();
      break;
    case V_EDITAR:    confirmarEdicion(); g_vista = V_MENU; pitidoOk(); break;
    case V_MANUAL: case V_ENSAYO: case V_PID: case V_CICLO: break;  // solo con larga

    case V_CONFIRMAR:
      if (g_siNo == 1) { g_puntoChecklist = 0; g_siNo = 0; g_vista = V_CHECKLIST; }
      else               g_vista = V_MENU;
      pitidoOk();
      break;

    case V_CHECKLIST:
      if (g_siNo != 1) { cancelarArranque("punto no confirmado"); pitidoOk(); break; }
      g_puntoChecklist++;
      g_siNo = 0;
      if (g_puntoChecklist >= N_CHECKLIST) {
        if (g_arrancandoPID) lanzarCiclo(); else lanzarEnsayo();
      } else pitidoOk();
      break;

    case V_MENU: {
      const ItemMenu &it = MENUS[g_menuActual].items[g_cursor[g_menuActual]];
      if (it.tipo == IT_VALOR) {
        if (it.valor == &g_ventanaEditable) g_ventanaEditable = g_ventanaMs;
        g_itemEditando = &it;
        g_vista = V_EDITAR;
      } else if (it.tipo == IT_SUBMENU) {
        g_menuActual = it.arg;
      } else switch (it.arg) {
        case AC_MANUAL:     g_vista = V_MANUAL;     break;
        case AC_INFO:       g_vista = V_INFO;       break;
        case AC_RESULTADOS: g_vista = V_RESULTADOS; break;
        case AC_CALIBRAR:   g_vista = V_CALIBRACION; break;
        case AC_GUARDAR:    g_par.ventanaMs = (uint32_t)g_ventanaEditable;
                            guardarParametros();    break;
        case AC_BUZZER:     pitidoOk(); delay(300); pitidoAviso(); break;
        case AC_VOLVER:     subirNivel();           break;
        case AC_INICIAR_ENSAYO:
          if (!g_sensorValido) { pitidoFalla(); break; }
          g_arrancandoPID = false; g_siNo = 0; g_vista = V_CONFIRMAR;
          break;
        case AC_INICIAR_PID:
          if (!g_sensorValido) { pitidoFalla(); break; }
          g_arrancandoPID = true;  g_siNo = 0; g_vista = V_CONFIRMAR;
          break;
      }
      pitidoOk();
      break;
    }
  }
}

void tareaUI(void *pv) {
  const TickType_t periodo = pdMS_TO_TICKS(PERIODO_SONDEO_MS);
  TickType_t ultima = xTaskGetTickCount();
  uint32_t proxDibujo = 0, proxReintento = 0;
  bool sucia = true;

  for (;;) {
    int giro = leerGiro();
    if (giro) { aplicarGiro(giro); sucia = true; }
    int pulso = leerPulsador();
    if (pulso) { aplicarPulsacion(pulso); sucia = true; }

    uint32_t ahora = millis();
    if (sucia || (long)(ahora - proxDibujo) >= 0) {
      refrescarPantalla();
      proxDibujo = ahora + PERIODO_DIBUJO_MS;
      sucia = false;
    }
    if (!g_pantallaOk && (long)(ahora - proxReintento) >= 0) {
      proxReintento = ahora + 3000;
      if (iniciarPantalla(1)) { pitidoOk(); sucia = true; }
    }
    vTaskDelayUntil(&ultima, periodo);
  }
}

// ---------------------------------------------------------------------------
// CONSOLA
// ---------------------------------------------------------------------------
void mostrarEstado() {
  const char *e = g_estado == EST_FALLA  ? "FALLA"
                : g_estado == EST_ENSAYO ? "ENSAYO"
                : g_estado == EST_PID    ? (g_enRampa ? "RAMPA" : "PID")
                : g_estado == EST_CICLO  ? nombreFase(g_fase) : "IDLE";
  Serial.println(F("# --- ESTADO ---"));
  Serial.printf("# Estado      : %s %s\n", e, g_motivoFalla);
  Serial.printf("# Salida      : %s  duty %d %% (real %.2f %%)\n",
                g_salidaArmada ? "ACTIVA" : "EN REPOSO", g_dutyComandado, g_dutyReal);
  Serial.printf("# Temperatura : %.3f C corregida  (cruda %.3f, offset %+.2f)\n",
                g_temperatura, g_tCruda, g_par.offsetC);
  Serial.printf("# Sensor      : [%s] %s\n", g_sensorValido ? "OK" : "NO VALIDA", g_avisoSensor);
  Serial.printf("# Limite abs. : %.0f C\n", T_LIMITE_ABSOLUTO);
  Serial.printf("# Potencia    : %.0f W / %.0f A (medido, sec.1.1)\n", POTENCIA_W, CORRIENTE_A);
  Serial.printf("# Setpoint    : %.1f C   F0 objetivo %.0f min\n",
                g_par.setpoint, g_par.f0Objetivo);
  Serial.printf("# Ensayo cfg  : %d %% / %d min\n",
                (int)g_par.dutyEnsayo, (int)g_par.minutosEnsayo);
  Serial.printf("# Muestras    : %u\n", g_nMuestras);
  Serial.printf("# PI          : Kp=%.2f Ki=%.4f  err=%+.2f  sal=%.1f%%\n",
                g_par.kp, g_par.ki, g_errorPID, g_salidaPID);
  Serial.printf("# Ebullicion  : %.2f C a %.0f m\n", puntoEbullicion(), ALTITUD_M);
  if (g_estado == EST_CICLO)
    Serial.printf("# Ciclo       : F0 %.2f / %.0f min  fase %s  total %lu:%02lu\n",
                  g_F0, g_par.f0Objetivo, nombreFase(g_fase),
                  (unsigned long)(segundosCiclo() / 60),
                  (unsigned long)(segundosCiclo() % 60));
  Serial.printf("# Relevo PID  : %.1f C   precarga %.1f %%\n",
                g_par.setpoint - ENTREGA_PID_C,
                DUTY_POR_GRADO * (g_par.setpoint - T_AMBIENTE_C));
  if (g_estado == EST_ENSAYO || g_estado == EST_PID)
    Serial.printf("# En curso    : %lu s de %lu\n",
                  (unsigned long)(millis() - g_inicioEnsayo)/1000,
                  (unsigned long)(g_duracionEnsayoMs/1000));
}

// Lee y decodifica el estado de la cadena de medicion en el momento
// --- Acceso directo a los registros del MAX31865 ---------------------------
// La libreria Adafruit tiene readRegister8 en privado, asi que aqui va un
// acceso propio. Sirve para distinguir "el chip no responde" de "el chip
// responde pero no circula corriente por la RTD", que son averias distintas.
// El MAX31865 usa SPI modo 1 (CPOL=0, CPHA=1).
uint8_t leerReg8(uint8_t addr) {
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(addr & 0x7F);
  uint8_t v = SPI.transfer(0xFF);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return v;
}

void escribirReg8(uint8_t addr, uint8_t val) {
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

void cmdRegistros() {
  Serial.println(F("# --- REGISTROS DEL MAX31865 ---"));
  const char *nombres[] = { "Config", "RTD MSB", "RTD LSB", "HFT MSB",
                            "HFT LSB", "LFT MSB", "LFT LSB", "Fallas" };
  for (uint8_t a = 0; a <= 7; a++)
    Serial.printf("#  0x%02X %-8s = 0x%02X\n", a, nombres[a], leerReg8(a));

  // Prueba de escritura: si el chip acepta lo que se le escribe, el bus SPI
  // funciona en ambos sentidos y el problema es analogico, no de comunicacion.
  uint8_t original = leerReg8(0x00);
  escribirReg8(0x00, 0xD1);              // VBIAS + auto + 3 hilos + 60 Hz
  delay(5);
  uint8_t leido = leerReg8(0x00);
  escribirReg8(0x00, original);

  Serial.printf("# Prueba de escritura: se escribio 0xD1, se leyo 0x%02X\n", leido);
  if (leido == 0xD1)
    Serial.println(F("#   [OK] SPI escribe y lee. El chip responde bien."));
  else if (leido == 0x00)
    Serial.println(F("#   [X] Solo se leen ceros: MISO muerto o chip sin alimentar."));
  else
    Serial.println(F("#   [?] El chip no conserva lo escrito. Revisar CS y alimentacion."));
}

// Mantiene VBIAS encendido para poder medir con el multimetro. Sin esto la
// corriente solo circula durante los 65 ms de conversion y no da tiempo.
void cmdBias(bool on) {
  uint8_t cfg = leerReg8(0x00);
  if (on) cfg |=  0x80; else cfg &= ~0x80;
  escribirReg8(0x00, cfg);
  Serial.printf("# VBIAS %s (config = 0x%02X)\n", on ? "ENCENDIDO" : "apagado",
                leerReg8(0x00));
  if (on) {
    Serial.println(F("# Ahora mide con el multimetro en DC:"));
    Serial.println(F("#   a) Entre RTD+ y RTD- (sobre la resistencia de prueba)"));
    Serial.println(F("#      Con 130 ohm deberia haber decenas de mV."));
    Serial.println(F("#      Si mide 0 mV, no circula corriente de excitacion."));
    Serial.println(F("#   b) Entre los extremos de Rref (430 ohm) en la placa"));
    Serial.println(F("#      Ahi SIEMPRE deberia haber tension si el chip vive."));
    Serial.println(F("# Cuando termines: comando 'bias off'"));
  }
}

void cmdDiag() {
  uint16_t raw = termo.readRTD();
  uint8_t  f   = termo.readFault();
  if (f) termo.clearFault();

  Serial.println(F("# --- DIAGNOSTICO DEL SENSOR ---"));
  Serial.printf("# RTD crudo   : %u\n", raw);
  Serial.printf("# Resistencia : %.3f ohm\n", (raw / 32768.0) * RREF);
  Serial.printf("# Temperatura : %.3f C\n", termo.calculateTemperature(raw, RNOMINAL, RREF));
  Serial.printf("# Sensor valido: %s  %s\n", g_sensorValido ? "SI" : "NO", g_avisoSensor);

  if (f) {
    Serial.printf("# Registro de fallas: 0x%02X -> %s\n", f, textoFalla(f));
    Serial.println(F("# Nota: este registro es intermitente con cable largo."));
    Serial.println(F("# La validez se decide por la lectura, no por el flag."));
  } else {
    Serial.println(F("# Registro de fallas: limpio."));
  }
}

// Ajuste de parametros por consola. Util cuando el PC esta conectado: evita
// tener que girar el encoder decimal a decimal para valores como Ki = 0.002.
bool fijarParametro(const String &c) {
  struct Ajustable { const char *nombre; float *valor; float lo, hi; uint8_t dec; };
  static Ajustable P[] = {
    { "kp",   &g_par.kp,             0.0f,  100.0f, 3 },
    { "ki",   &g_par.ki,             0.0f,   10.0f, 5 },
    { "kd",   &g_par.kd,             0.0f,  100.0f, 3 },
    { "sp",   &g_par.setpoint,      20.0f,  135.0f, 1 },
    { "min",  &g_par.minutosEnsayo,  5.0f,  240.0f, 0 },
    { "duty", &g_par.dutyEnsayo,     5.0f,  100.0f, 0 },
    { "f0",   &g_par.f0Objetivo,     1.0f,  120.0f, 0 },
  };
  for (uint8_t i = 0; i < sizeof(P) / sizeof(Ajustable); i++) {
    String pre = String(P[i].nombre) + " ";
    if (!c.startsWith(pre)) continue;
    float v = c.substring(pre.length()).toFloat();
    if (v < P[i].lo || v > P[i].hi) {
      Serial.printf("# [!] %s fuera de rango (%.1f a %.1f)\n", P[i].nombre, P[i].lo, P[i].hi);
      return true;
    }
    *P[i].valor = v;
    guardarParametros();
    Serial.printf("# %s = %.*f  (guardado en NVS)\n", P[i].nombre, P[i].dec, v);
    return true;
  }
  return false;
}

void ayuda() {
  Serial.println(F("# Comandos: estado | diag | cal <C> | calibrar | manual | escalon |"));
  Serial.println(F("#           sonido | probarfalla | regs | bias on|off | armar |"));
  Serial.println(F("#           desarmar | parar | resultados | volcar | borrar | reset | help"));
  Serial.println(F("# Parametros: kp | ki | kd | sp | min | duty | f0  <valor>"));
  Serial.println(F("# El ensayo se lanza desde el MENU con el encoder."));
}

void procesarComando(String c) {
  c.trim(); c.toLowerCase();
  if (!c.length()) return;

  if (c == "armar") {
    if (g_estado == EST_FALLA) { Serial.println(F("# [!] En FALLA. Usa reset.")); return; }
    if (!g_sensorValido)       { Serial.println(F("# [!] Sensor sin lectura valida.")); return; }
    g_salidaArmada = 1;
    Serial.println(F("# *** SALIDA ARMADA ***"));
    pitidoOk();
  }
  else if (c == "desarmar")   desarmar("comando del operador");
  else if (c == "parar") {
    if (g_estado == EST_CICLO) {
      desarmar("ciclo detenido por consola"); g_estado = EST_IDLE; g_pedirFlush = true;
    }
    else if (g_estado == EST_ENSAYO || g_estado == EST_PID) terminarEnsayo("parada manual");
    else Serial.println(F("# No hay ensayo en curso."));
  }
  else if (c == "estado")     mostrarEstado();
  else if (c == "diag")       cmdDiag();
  // La consola abre las pantallas que se retiraron del menu. Sigue haciendo
  // falta un PC para llegar a ellas, que es justo el control que se buscaba.
  else if (c == "calibrar")   { g_vista = V_CALIBRACION;
                                Serial.println(F("# Pantalla de calibracion abierta en el equipo.")); }
  else if (c == "manual")     { g_vista = V_MANUAL;
                                Serial.println(F("# Modo manual abierto. Usa 'armar' para habilitar.")); }
  else if (c == "escalon")    { if (!g_sensorValido) { Serial.println(F("# [!] Sensor sin lectura valida.")); }
                                else { g_arrancandoPID = false; g_siNo = 0;
                                       g_vista = V_CONFIRMAR;
                                       Serial.println(F("# Ensayo de escalon: confirma en el equipo.")); } }
  else if (c == "sonido")     { pitidoOk(); delay(300); pitidoAviso(); delay(300); avisoAccion(); }
  // Un enclavamiento que no se puede probar es un enclavamiento que no se
  // sabe si funciona. El limite absoluto no se puede falsear con "cal"
  // -- el offset esta topado en +/-3 C y se vigila fmaxf(t, cruda) -- asi
  // que esta es la unica forma de verificar el latch sin calentar la olla
  // de verdad. Queda en consola, fuera del alcance del operador, y el
  // motivo dice PRUEBA para que no se confunda con un evento real.
  else if (c == "probarfalla") {
    if (g_estado != EST_IDLE) {
      Serial.println(F("# Solo en reposo. Pare el ciclo primero."));
      return;
    }
    Serial.println(F("# Lanzando falla critica de PRUEBA. Deberia:"));
    Serial.println(F("#   1. mostrar ** FALLA GRAVE ** en la pantalla"));
    Serial.println(F("#   2. NO borrarse con pulsacion larga"));
    Serial.println(F("#   3. seguir ahi despues de cortar la alimentacion"));
    entrarEnFallaCritica("PRUEBA de enclavamiento (consola)");
  }
  else if (c.startsWith("cal ")) {
    float v = c.substring(4).toFloat();
    if (v > OFFSET_MAX_C || v < -OFFSET_MAX_C) {
      Serial.printf("# [!] Offset fuera de +/-%.1f C. Un error mayor no se tapa,\n",
                    OFFSET_MAX_C);
      Serial.println(F("#     se investiga: Rref, modo 3 hilos, contacto de la sonda."));
    } else {
      g_par.offsetC = v; guardarParametros();
      Serial.printf("# Offset = %+.2f C. Cruda %.3f -> corregida %.3f\n",
                    v, g_tCruda, g_tCruda + v);
    }
  }
  else if (c == "regs")       cmdRegistros();
  else if (c == "bias on")    cmdBias(true);
  else if (c == "bias off")   cmdBias(false);
  else if (c == "resultados") imprimirResultados(g_res);
  else if (c == "volcar")     volcarCSV();
  else if (c == "borrarres"){ g_res = {0,0,0,0,false}; guardarResultados();
                              Serial.println(F("# Resultados borrados.")); }
  else if (c == "borrar")   { LittleFS.remove(RUTA_CSV); g_nMuestras = 0;
                              Serial.println(F("# Ensayo borrado.")); }
  else if (c == "reset") {
    if (g_estado != EST_FALLA) { Serial.println(F("# No hay falla activa.")); return; }
    bool era = g_fallaCritica;
    g_estado = EST_IDLE; g_motivoFalla[0] = '\0'; g_vista = V_PRINCIPAL;
    g_fallaCritica = false;
    prefs.putBool("fcrit", false);
    prefs.remove ("fmot");
    Serial.printf("# *** FALLA%s RECONOCIDA. Salida sigue DESARMADA. ***\n",
                  era ? " CRITICA" : "");
    pitidoOk();
  }
  else if (fijarParametro(c)) { }
  else if (c == "help") ayuda();
  else { Serial.printf("# Comando desconocido: %s\n", c.c_str()); ayuda(); }
}

// ---------------------------------------------------------------------------

void setup() {
  // §7.2: estado seguro antes que nada.
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, LOW);

  Serial.begin(115200);
  delay(500);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);

  Serial.println(F("# ================================================"));
  Serial.println(F("#  PID AUTOCLAVE 75X - Paso 4: caracterizacion"));
  Serial.println(F("#  *** ENERGIZA LA RESISTENCIA REAL ***"));
  Serial.println(F("#  Autonomo: no necesita PC para el ensayo."));
  Serial.println(F("# ================================================"));

  cargarParametros();
  g_ventanaEditable = g_par.ventanaMs;
  restaurarFallaCritica();

  if (!LittleFS.begin(true)) Serial.println(F("# [!] LittleFS no disponible."));

  termo.begin(MAX31865_3WIRE);
  termo.enable50Hz(FILTRO_50HZ);
  termo.clearFault();

  iniciarPantalla(10);
  iniciarEncoder();

  esp_task_wdt_config_t wdt = {};
  wdt.timeout_ms = WDT_TIMEOUT_MS;
  wdt.idle_core_mask = 0;
  wdt.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);

  xTaskCreatePinnedToCore(tareaSalida, "salida", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(tareaSensor, "sensor", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(tareaUI,     "ui",     8192, NULL, 1, NULL, 0);

  melodiaArranque();
  ayuda();
}

void loop() {
  static int  estadoPrevio = EST_IDLE;

  if (Serial.available()) procesarComando(Serial.readStringUntil('\n'));

  // Escritura en flash fuera de las tareas criticas
  if (g_pedirFlush) { g_pedirFlush = false; escribirCSV(); }

  if (g_estado == EST_FALLA && estadoPrevio != EST_FALLA) pitidoFalla();
  if (estadoPrevio == EST_ENSAYO && g_estado == EST_IDLE) {
    melodiaFin();
    g_vista = V_RESULTADOS;
  }
  // La melodia suena cuando se cumple el F0, que es el hito real del
  // proceso. Antes estaba atada a que el operador descartara el aviso en
  // F_TERMINADO: sonaba al cerrar el "NO ABRIR" y se quedaba muda al
  // cerrar el "DESPRESURIZADO", justo al reves de lo que corresponde. Y
  // como reponia g_vista = V_CICLO, la pulsacion larga no cerraba nada.
  if (g_pedirMelodiaFin) { g_pedirMelodiaFin = false; melodiaFinCiclo(); }
  estadoPrevio = g_estado;

  delay(20);
}
