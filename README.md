# Controladora PID para Autoclave All American 75X

Firmware ESP32 para convertir un autoclave **All American Model 75X** en un
esterilizador de proceso validado, con control PID y **terminación por letalidad
acumulada (F₀)** en lugar de temporizador fijo.

Desarrollado para esterilización de sustrato en un laboratorio de microbiología
en **Medellín, Colombia (1.495 m sobre el nivel del mar)** — una altitud que, como
se verá, cambia bastante las cosas.

---

## Por qué F₀ y no un temporizador

La letalidad de un proceso de esterilización depende **exponencialmente** de la
temperatura. Un ciclo de 20 minutos a 118 °C no equivale a uno de 20 minutos a
121 °C: da menos de la mitad de letalidad.

Este firmware acumula, cada segundo:

```c
F0 += pow(10.0, (T - 121.1) / 10.0) / 60.0;   // z = 10 °C
```

y **termina cuando F₀ alcanza el objetivo**, dure lo que dure. Las caídas de
temperatura y las oscilaciones se integran solas: no hay conteo que reiniciar.

### El caso que lo justifica

A 1.495 m la presión atmosférica es 84.6 kPa, no 101.3. La escala de temperatura
impresa en el manómetro del autoclave está calculada para el nivel del mar:

| Manómetro | Temperatura **real** aquí | La escala dice |
| ---: | ---: | ---: |
| 15 psi | 118.5 °C | 121 °C |
| **17.5 psi** | **121.1 °C** | ~124 °C |

Antes de este proyecto el equipo se operaba a 16 psi creyendo estar a 121 °C.
La temperatura real eran **119.6 °C**, y un ciclo de 20 minutos daba **F₀ ≈ 14**
en lugar de 20 — un 30 % por debajo, sin forma de saberlo.

Con F₀ el sistema es inmune a ese error: mide la temperatura real y el ciclo
dura lo que tenga que durar.

---

## Resultados medidos

Ciclo completo validado, desde purga hasta despresurización:

| | |
| :--- | ---: |
| Sobrepaso sobre la consigna | **+0.29 °C** |
| Estabilidad en meseta | **sd 0.012 °C** |
| F₀ alcanzado | 20.00 min |
| Duración total | 56 min |
| Verificación cruzada con manómetro | 18 psi medidos vs 17.9 predichos |

### Cómo se llegó ahí

El sobrepaso bajó de +3.13 °C a +0.29 °C en tres iteraciones, cada una
atacando una causa distinta:

| Corrida | Cambio | Sobrepaso |
| :--- | :--- | ---: |
| 1ª | PID desde el arranque, integrador desde cero | +3.13 °C |
| 2ª | Rampa en lazo abierto + relevo del PID cerca de la consigna | +1.06 °C |
| 3ª | Precarga del integrador + banda de integración | **+0.29 °C** |

El problema de fondo: en la consigna el término proporcional vale cero, así que
**toda la potencia de mantenimiento tiene que salir del integrador**. Partiendo de
cero, en una planta con τ de 84 minutos, el integrador se pasa de largo mucho
antes de que la temperatura responda. Ningún esquema de anti-windup lo evita —
acotan el integrador, no impiden que tienda al valor que mantiene la salida
saturada.

La solución fue no darle al PID esa tarea: rampa en lazo abierto hasta cerca de
la consigna, y ahí el PID entra **ya precargado** con el duty de régimen.

---

## Hardware

| Componente | Detalle |
| :--- | :--- |
| MCU | ESP32 DevKit V1, 38 pines |
| Sensor | PT100 clase A, 3 hilos, en termowell |
| Acondicionador | MAX31865 (CJMCU-865), Rref 430 Ω |
| Actuador | SSR-25 DA zero-cross, PWM lento de 2 s |
| Carga | 1.680 W / 14 A a 120 VAC (medida con pinza) |
| Interfaz | OLED SSD1306 I²C + encoder KY-040 por PCNT |
| Seguridad | Termostato bimetálico 130 °C sobre contactor, fusible, válvula de seguridad |

### Arquitectura FreeRTOS

| Tarea | Prioridad | Periodo | Puede bloquearse |
| :--- | :---: | :--- | :--- |
| `tareaSalida` | 3 | 5 ms | **No** — solo conmuta el SSR |
| `tareaSensor` | 2 | 1 s | Sí — 375 ms de conversiones |
| `tareaUI` | 1 | 10 ms | Sí |

La separación no es decorativa: `Adafruit_MAX31865::readRTD()` bloquea 75 ms por
conversión, y con mediana de 5 son 375 ms. Dentro del mismo bucle que genera el
PWM, la ventana de 2 s se desfasaría un 19 %.

---

## Ciclo de esterilización

```
CHECKLIST      6 puntos, confirmación individual con el encoder
     ↓
CALENTANDO     100 %, válvula de purga abierta
     ↓ ♪
PURGA          7 min de vapor continuo barriendo el aire
     ↓ ♪♪♪
CERRAR VALVULA aviso insistente. El cierre se detecta por física:
               la temperatura sube 2 °C al sellarse la olla
     ↓
RAMPA          100 % hasta consigna − 10 °C
     ↓
ESTERILIZANDO  PID + acumulación de F₀
     ↓ ♪
ENFRIANDO      salida cortada, aún con presión
     ↓ ♪♪♪
DESPRESURIZADO aviso al bajar de 90 °C
```

**La purga no es opcional.** Es el modo de falla número uno en autoclaves
domésticos: el aire atrapado forma bolsas donde el vapor no penetra, y la sonda
del termowell no se entera de nada.

**El aviso de despresurización no autoriza a abrir.** Dice *"verifique el
manómetro en cero"*. La temperatura es una inferencia; el manómetro es una
medida directa. Y el umbral son 90 °C, no 100: a esta altitud, 100 °C todavía
son 2.4 psi.

---

## Seguridad

| Capa | Umbral | Tipo |
| :--- | :--- | :--- |
| Válvula de seguridad de la olla | — | Mecánica, primaria |
| Termostato bimetálico + contactor | 130 °C | Mecánica, independiente del firmware |
| Límite absoluto en firmware | 128 °C | Corte incondicional → FALLA |
| Validez del sensor | RTD abierta, en corto, salto > 5 °C/s | → FALLA |
| Detección de marcha en seco | ver nota | → FALLA |
| Watchdog | 8 s | sobre la tarea de salida |

`digitalWrite(PIN_SSR, LOW)` es la primera instrucción de `setup()`, antes
incluso de `Serial.begin()`.

### La FALLA tiene dos niveles

Porque las causas no son igual de graves, y tratarlas igual significó durante
un tiempo que superar los 128 °C se podía borrar con una pulsación larga:

| | Causas | Cómo se sale |
| :--- | :--- | :--- |
| **Recuperable** | Marcha en seco, tiempo máximo agotado | Pulsación larga. La causa está afuera del equipo y el operador la corrige |
| **Crítica** | Límite absoluto superado, sensor ciego con la salida energizada | Solo por consola. Falló el hardware |

La crítica **queda grabada en NVS**: cortar la alimentación no la borra. Si el
SSR quedó conduciendo sin orden, el ciclo siguiente calentaría sin control, así
que el equipo se queda detenido hasta que lo revise alguien con consola.

### Las protecciones se pueden probar

Una protección configurada y no probada es una suposición. Dos comandos de
consola, fuera del alcance del operador, permiten reverificarlas después de
cada cambio de firmware sin calentar la olla:

```
probarfalla   enclava una falla crítica de prueba
              -> pantalla ** FALLA GRAVE **, no cede a la pulsación larga,
                 sobrevive al corte de alimentación, se borra con 'reset'

probarwdt     cuelga la tarea de salida a propósito (corta el SSR antes)
              -> el watchdog debe reiniciar la placa en 8 s y el arranque
                 debe reportar: ARRANQUE TRAS WATCHDOG DE TAREA
```

El firmware informa el motivo de cada arranque. Un `CAIDA DE TENSION` o un
`WATCHDOG DE TAREA` en el log explica por sí solo un F₀ que no cerró.

> **Nota sobre la marcha en seco:** el detector ingenuo («salida al 100 % durante
> X minutos sin subir Y grados») **da falso positivo durante la purga**, donde el
> estancamiento en la ebullición es normal. Aquí se desactiva en la banda de
> ±5 °C alrededor del punto de ebullición local, y se reactiva por encima — que
> es donde la marcha en seco realmente se manifiesta, con una subida rápida.

---

## Estructura

```
CONTEXTO.md                  Especificación, decisiones y bitácora técnica
CHECKLIST_PRIMER_ENSAYO.md   Pre-flight imprimible para el primer ensayo
GUIA_SIN_PC.md               Operación autónoma desde el encoder
subir.ps1                    Compila, sube y monitoriza (autodetecta puerto)

test_comunicacion/           Paso 0: verificación del enlace
01_validacion_pt100/         Paso 1: cadena de medición
02_pwm_lento/                Paso 2: etapa de salida
03_ui_encoder_oled/          Paso 3: interfaz
04_caracterizacion/          Firmware completo: caracterización, PID y ciclo

herramientas/registrar.py    Registro y análisis FOPDT desde PC
ensayos/                     Curvas registradas
```

`CONTEXTO.md` documenta cada decisión con los datos que la respaldan, incluidas
las que resultaron equivocadas y por qué. Es el documento a leer antes de tocar
nada.

---

## Compilar

```powershell
.\subir.ps1 04_caracterizacion
```

Requiere `arduino-cli`, el core `esp32:esp32` y las librerías
`Adafruit_MAX31865`, `U8g2`.

> La subida va a **115200 baudios**, no a los 921600 por defecto: con el montaje
> definitivo la velocidad alta corta la escritura a mitad.

---

## Estado

| Paso | Estado |
| :--- | :--- |
| 0–2. Comunicación, sensor, salida | ✅ |
| 3. Interfaz | ✅ |
| 4. Caracterización de planta | ✅ |
| 5. Control PID | ✅ |
| 6. Ciclo con F₀ | ✅ |
| 7. Capas de seguridad | ✅ las tres capas verificadas en hardware |
| 8. Gabinete y validación de penetración | ⬜ |

**Pendiente crítico:** validación con sonda de penetración en el centro de la
carga. 121 °C en el termowell no significa 121 °C en el centro del sustrato: la
arena es mal conductor y constituye carga densa. Esa medida decidirá si el F₀
objetivo debe subir por encima de 20.

---

*BY_Oquendo*
