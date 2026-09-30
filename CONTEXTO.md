# CONTEXTO DE PROYECTO: Controladora PID para Autoclave All American 75X

**Propietario:** Sr. Carlos
**Equipo:** Autoclave All American Model No. 75X (Wisconsin Aluminum Foundry Co.)
**Aplicación:** Esterilización de arena / sustrato para cría de grillos
**Plataforma:** ESP32 DevKit V1 (**38 pines**), Arduino framework
**Actualizado:** Septiembre 2026

---

## 0. PRIORIDADES DEL PROYECTO (orden estricto)

Firmware para controlador de esterilización por vapor a presión. No es un simple
control de temperatura: es un proceso validado con requisitos sanitarios
(letalidad acumulada) y de seguridad física (recipiente a presión).

1. **Seguridad** — el equipo opera a 120 VAC y a ~2 bar de presión.
2. **Eficacia de esterilización** — un ciclo que "se ve bien" pero no esteriliza
   es peor que no tener ciclo.
3. **Precisión de control** — evitar sobrepasos que se traducen en sobrepresión.
4. **Usabilidad** — interfaz de un solo encoder.

Nada en el firmware puede omitir un requisito de la sección 7.

---

## 1. ESPECIFICACIONES ELÉCTRICAS Y TÉRMICAS

| Parámetro | Valor | Estado |
| :--- | :--- | :--- |
| Tensión de alimentación | 120 VAC, 60 Hz | Confirmado |
| Potencia nominal de placa | 1650 W | **Sin confirmar** |
| Corriente nominal de placa | 13.75 A | **Sin confirmar** |
| Lectura de resistencia en frío | 23.7 Ω → ~607 W / 5.06 A | **Sin confirmar** |
| Alojamiento de sonda | Termowell metálico integrado a la olla | Confirmado |

### 1.1 RESUELTO — escenario B, medido con pinza amperimétrica

**Medida real: 14 A a 120 VAC = ~1680 W.** Es el escenario B.

La lectura de 23.7 Ω en frío engañaba, exactamente como advertía esta sección:
la resistencia real en caliente es ~8.6 Ω. El coeficiente térmico del elemento
casi triplica su valor entre frío y régimen.

| Consecuencia | Valor |
| :--- | :--- |
| Fusible | **20 A** (uno de 10 A se funde en el primer ciclo) |
| Disipador del SSR | **CON ALETAS, obligatorio** — disipa ~17 W a 14 A |
| SSR-25 DA | Trabaja al 56 % de su rating |
| Cable de potencia | 10 AWG siliconado: sobrado |
| `POTENCIA_W` en firmware | 1680 W (v0.4.3) |
| Umbral de marcha en seco | Recalibrado: 5 min / 5 °C (antes 10 min) |

> ⚠️ **A 14 A, un SSR mal disipado se calienta hasta fallar — y los SSR fallan
> en CORTOCIRCUITO**, quedando permanentemente conducido. Ese es exactamente el
> escenario contra el que solo protege el termostato bimetálico. No energizar
> sin el disipador con aletas montado.

### 1.1.bis Redacción original (conservada como referencia)

Los dos valores de potencia son incompatibles: 1650 W implicarían ~8.7 Ω, no 23.7 Ω.

- **Escenario A — 607 W / 5.06 A:** SSR-25 al 20 % de capacidad, disipa ~8 W, la
  carcasa de aluminio del gabinete basta como disipador. Rampa a 121 °C: 45–60 min.
- **Escenario B — 1650 W / 13.75 A:** SSR al 55 % de rating, disipa 16–20 W.
  **Exige disipador con aletas.** Los SSR genéricos se degradan bastante antes de
  su valor de placa.

**Acción requerida:** medir con pinza amperimétrica en operación real. La medición
de resistencia en frío puede engañar (coeficiente térmico positivo, o se midió
solo un tramo).

**Impacto en firmware:** `POTENCIA_W` afecta las estimaciones de rampa y los
umbrales de detección de operación en seco (§7.2). Constante configurable en un
solo lugar.

---

## 2. LISTA DE MATERIALES

| Cant. | Componente | Especificación |
| :---: | :--- | :--- |
| 1 | Microcontrolador **ESP32** | DevKit V1, **38 pines** (ver §3.2) |
| 1 | Amplificador **MAX31865** | SPI, para RTD. **Verificar Rref = 430 Ω** |
| 1 | Sensor **PT100** | 3 hilos, vaina inox 4 o 6 mm según termowell |
| 1 | **SSR-25 DA** | Entrada 3–32 VDC, salida 24–380 VAC, zero-cross |
| 1 | Disipador para SSR | Con aletas si aplica escenario B (§1.1) |
| 1 | Pantalla **OLED 0.96" / 1.3"** | I²C, SSD1306 o SH1106 |
| 1 | **Encoder rotativo** con pulsador | Módulo KY-040 |
| 1 | Fuente **5 V / 1 A** aislada | Hi-Link HLK-PM01 o equivalente |
| 1 | **Termostato bimetálico NC** | Corte a **130 °C**, mínimo 16 A (KSD301) |
| 1 | **Resistencia 10 kΩ** | Pull-down para entrada del SSR — crítica |
| 1 | Portafusibles + fusible | 10 A o 20 A según §1.1 |
| 1 | Buzzer | Fin de ciclo y alarmas. **No arranca con los 3.3 V del GPIO** |
| 1 | Transistor NPN | Etapa de disparo del buzzer. TIP122 (validado) o 2N2222 |
| 1 | Resistencia 1 kΩ | Base del transistor del buzzer |
| 1 | Diodo **1N4148** | Antiparalelo al buzzer, contra el pico inductivo |
| 1 | Bornera de tierra | Chasis a tierra, obligatorio |
| 2 | Prensaestopas | Entrada de red y salida de cable PT100 |
| — | Pasta térmica sintética >150 °C | Termowell y acople del SSR |
| — | Cable de potencia **12 AWG** | THHN / TFFN / siliconado |
| — | Camisa de fibra de vidrio | Cable de la PT100 y **cable del termostato** |
| 1 | Abrazadera inox o fleje | Sujeción del termostato a la olla (§10.6) |
| — | Lana cerámica / fibra | Aislar el termostato del aire ambiente — **crítico** |

### 2.1 PT100 en lugar de termopar tipo K

Decisión tomada y confirmada. A 121 °C la PT100 clase A da ±0.3 °C frente a
±2.2 °C del tipo K. En un proceso donde la letalidad depende exponencialmente de
la temperatura, esa diferencia es significativa. **No revertir.**

---

## 3. DIAGRAMA DE CONEXIONES

```text
                         +----------------------------------+
                         |          ESP32 DevKit V1         |
                         |                                  |
 +----------------+      | GPIO 18 (SPI SCK)   <-> CLK      |
 |   MAX31865     |      | GPIO 19 (SPI MISO)  <-- SDO      |
 |    + PT100     |------| GPIO 23 (SPI MOSI)  --> SDI      |
 |    (3 hilos)   |      | GPIO 5  (CS)        --> CS       |
 +----------------+      |                                  |
                         |                                  |     +----------------+
                         | GPIO 22 (I2C SCL)   ------------>|---->|   OLED I2C     |
                         | GPIO 21 (I2C SDA)   <----------->|<--->|   SSD1306      |
                         |                                  |     +----------------+
                         |                                  |
                         | GPIO 25 (PWM lento) ------[R]----|--> SSR-25 borne 3 (+)
                         |                          10k     |    GND -> borne 4 (-)
                         |                           |      |
                         |                          GND     |
                         |                                  |     +----------------+
                         | GPIO 26 (Encoder CLK)            |     |    KY-040      |
                         | GPIO 27 (Encoder DT)             |-----|  ENCODER       |
                         | GPIO 14 (Encoder SW)             |     +----------------+
                         |                                  |
                         | GPIO 33 (Buzzer)                 |
                         +----------------------------------+
```

### 3.1 Correcciones respecto al diagrama original

**A. SDI / SDO estaban invertidos.** En el módulo MAX31865, `SDO` es la salida
del módulo y va a **MISO (GPIO19)**; `SDI` es la entrada y va a **MOSI (GPIO23)**.

**B. Falta pull-down de 10 kΩ en GPIO25 — crítico.** Durante arranque y reset los
GPIO quedan flotantes algunos ms. Sin pull-down a GND en la entrada del SSR el
SSR podría conducir en cada reinicio. Inaceptable en un recipiente a presión. La
resistencia va físicamente entre GPIO25 y GND, junto a la entrada del SSR.

**C. GPIO5 es pin de strapping.** Debe estar alto durante el arranque. El CS de
SPI está en reposo alto, por lo que normalmente funciona. Si hay arranques
erráticos, mover el CS a GPIO32 o GPIO33.

**D. Configuración obligatoria del MAX31865.**
- Verificar que `Rref` sea **430 Ω**. Muchos módulos genéricos traen 4300 Ω
  (versión PT1000) y darán lecturas absurdas.
- En Adafruit_MAX31865, configurar filtro de **60 Hz**, no 50.

**D.1 Cableado de la PT100 de 3 hilos (según documentación Adafruit).**

Identificar los hilos con multímetro: los dos que midan ~2 Ω entre sí son el par
del mismo color (mismo extremo del elemento); el tercero mide ~108 Ω contra
ellos a temperatura ambiente.

```
  Par del mismo color ──→ F+ y RTD+   (bloque de la DERECHA)
                          (da igual cuál va dentro y cuál fuera)

  Hilo suelto        ──→ F- o RTD-    (bloque de la IZQUIERDA)
                          (da igual la ranura; F- y RTD- van puenteados)
```

**Modificaciones físicas obligatorias de la placa:**
1. Puente **`2/3 Wire`**: cortar la pista fina de la posición de 2 hilos y soldar
   la gota en la posición de 3 hilos.
2. Puente junto al **bloque de terminales izquierdo**: soldarlo, o cortocircuitar
   F- con RTD- metiendo un trozo de cable en la bornera.

Sin estas dos modificaciones la placa sigue eléctricamente en 2 hilos y la
compensación de resistencia de línea no funciona.

### 3.2 La placa es de 38 pines, no de 30

Verificado sobre el hardware real. Las conexiones por GPIO del §3 siguen siendo
válidas; lo que cambia son las posiciones físicas.

⚠️ **Riesgo que el modelo de 30 pines no tiene.** Las seis patillas extra
rotuladas **`D0 D1 D2 D3 CMD CLK`** (GPIO 6–11) están cableadas a la memoria
flash interna del ESP32 y **no se pueden usar para nada**. El pin `5V` queda
justo al lado de `CMD` en la esquina inferior izquierda: cuidado al conectar la
alimentación.

Los **cuatro pines GND** de la placa están todos unidos al mismo nodo
internamente. Se puede usar cualquiera, o varios a la vez.

### 3.3 Etapa de disparo del buzzer — necesaria

Los 3.3 V de GPIO33 **no bastan** para excitar el buzzer directamente. Etapa
validada en el hardware:

```
   5V ─────────────────→ Buzzer (+)
                         Buzzer (−) ──→ Pin 2  (COLECTOR / aleta metálica)
   GPIO33 ──[1 kΩ]─────→ Pin 1  (BASE)
   GND ────────────────→ Pin 3  (EMISOR)

   1N4148 en paralelo con el buzzer, cátodo (raya) hacia el +
```

**Pinout del TIP122 en TO-220, visto de frente** (cara impresa hacia el
observador, patillas hacia abajo): **1 = BASE, 2 = COLECTOR, 3 = EMISOR**. La
aleta metálica es el colector.

> Error cometido y corregido durante el montaje: se conectó el pin 1 a GND
> creyendo que era el emisor. Con la base cortocircuitada a masa el transistor no
> puede conducir nunca y el buzzer no suena, sin ningún síntoma que lo delate.

El TIP122 es un Darlington de 5 A, muy sobredimensionado para un buzzer y con
caída colector-emisor de 1–2 V. Funciona, pero un 2N2222 o BC547 sobra si se
quiere optimizar.

---

---

## 4. MONTAJE Y DISIPACIÓN

1. **SSR:** limpiar la superficie de contacto, capa uniforme y delgada de pasta
   térmica, atornillar con M4/M5 y arandelas de presión. Si aplica escenario B
   (§1.1), disipador con aletas y verificar temperatura de la carcasa del SSR en
   el primer ciclo completo.
2. **Sonda PT100:** punto de pasta térmica al fondo del termowell, introducir
   hasta hacer contacto con el fondo, asegurar el cable contra vibración.
3. **Termostato de respaldo:** en serie con la fase de 120 VAC de la resistencia.
   Si el SSR falla en corto, corta mecánicamente. Ajustado a **130 °C** (§7.1).
4. **Cableado:** el cable de la PT100 separado del cableado de potencia. Tierra de
   chasis obligatoria. Fusible en la fase, antes de todo lo demás.

---

## 5. CICLO DE ESTERILIZACIÓN — MÁQUINA DE ESTADOS

### Fase 0 — CHECKLIST PREVIO
Pantalla obliga a confirmar con el encoder, uno por uno:
- Nivel de agua correcto (resistencia completamente sumergida)
- Válvula de seguridad libre y sin obstrucción
- Tapa asegurada y sellada
- Carga distribuida en capas ≤ 5 cm

No se energiza el SSR hasta completar el checklist.

### Fase 1 — PURGA DE AIRE (obligatoria)
**Modo de falla número uno en autoclaves domésticas.** Si queda aire atrapado se
forman bolsas donde el vapor no penetra y la temperatura local queda muy por
debajo de 121 °C, aunque la PT100 en el termowell lea correctamente. Sin purga,
todo el control PID es decorativo.

- Calentar al 100 % con la válvula de purga **abierta**.
- Al alcanzar ~100 °C, sostener 7–10 min con salida continua de vapor.
- Pantalla instruye: *"Mantenga la válvula abierta hasta ver chorro continuo de
  vapor"*, luego *"Cierre la válvula y confirme"*.
- Confirmación manual con el encoder antes de pasar de fase.

### Fase 2 — RAMPA
- Salida al 100 % **solo hasta 105–110 °C**, no hasta 121 °C.
- A partir de ahí, control al PID. Ir al 100 % hasta el setpoint provoca
  sobrepaso por inercia térmica del agua y el aluminio, y en un recipiente a
  presión el sobrepaso es sobrepresión.

### Fase 3 — MESETA con acumulación de F₀
- Setpoint 121 °C, control PID con PWM lento.
- **No usar temporizador simple.** Acumular letalidad:

```c
F0 += pow(10.0, (T - 121.1) / 10.0) * dt_minutos;   // z = 10 °C
```

- Terminar cuando `F0 >= 20`.
- Las caídas breves de temperatura y las oscilaciones se integran
  automáticamente; no hay que reiniciar ningún conteo.
- Mostrar F₀ acumulado y estimación de tiempo restante.

### Fase 4 — FIN DE CICLO
- Deshabilitar salida del SSR.
- Buzzer + mensaje en OLED.
- Despresurización natural. **No abrir hasta presión cero.**

### 5.1 Penetración en arena
La arena es mal conductor térmico y constituye una carga densa. 121 °C en el
termowell no significa 121 °C en el centro del recipiente de sustrato.

- Capas de máximo 4–5 cm. **El laboratorio trabaja con 20 cm** (§10.16):
  no invalida el proceso, pero hace imprescindible la validación de
  penetración para fijar el F₀ objetivo correcto.
- Sustrato **húmedo**: el vapor no penetra material seco.
- Tiempos efectivos de 45–60 min en lugar de 20.
- Validación recomendada una vez: segunda sonda enterrada en el centro de la
  carga para medir el retraso real y calibrar el requisito de F₀.

---

## 6. ESTRATEGIA DE CONTROL

- **Salida:** PWM lento (time-proportional), ventana de **2 s**. El SSR es
  zero-cross; no admite PWM rápido.
- **Algoritmo:** probablemente basta un **PI**. El término derivativo en un
  proceso con vapor y cambio de fase suele generar más problemas que soluciones.
  Si se usa D, aplicarlo sobre la medición, no sobre el error.
- **Anti-windup obligatorio.** Saturación de salida 0–100 %.
- **Tiempo de muestreo fijo:** 500 ms – 1 s.
- **Autotune:** librería `sTune`, ejecutado en la zona de **110–121 °C**, nunca
  desde temperatura ambiente. La dinámica antes y después de la ebullición es
  completamente distinta.
- **Filtrado de lectura:** mediana + IIR simple sobre la señal de la PT100.
- Guardar parámetros PID en NVS con `Preferences`.

### 6.1 Presupuesto de tiempo del lazo — MEDIDO

`Adafruit_MAX31865::readRTD()` bloquea **~75 ms por conversión** (10 ms de
estabilización de bias + 65 ms de conversión) y lo hace incondicionalmente:
llamar a `autoConvert(true)` no ayuda porque `readRTD()` fuerza el modo one-shot
igualmente, y `readRegister16()` es privado, así que no se puede puentear sin
escribir un driver propio.

Medido en el hardware real (comando `tiempo` del sketch del paso 1):

| Operación | Latencia | % de un periodo de 1 s |
| :--- | ---: | ---: |
| `readRTD()` | 74.96 ms | 7.5 % |
| Mediana de 3 | 225.0 ms | **22.5 %** |
| Mediana de 5 | ~375 ms | 37.5 % |

**Decisiones derivadas:**
- **Una sola conversión por muestra.** `temperature()` llama internamente a
  `readRTD()`, así que pedir `readRTD()` y `temperature()` cuesta 150 ms sin
  ganar nada. Usar `readRTD()` una vez y derivar resistencia y temperatura de ese
  valor con `calculateTemperature()` (público, solo aritmética).
- **Mediana de 3, no de 5.** Con sd cruda de 0.039 °C la de 5 no aporta; la de 3
  ya rechaza picos y deja 77 % del periodo libre.
- **Periodo de control de 1 s.** Sobrado para un proceso térmico con constante de
  tiempo de decenas de minutos.
- En ESP32 `delay()` llama a `vTaskDelay()`, así que ese bloqueo **cede la CPU**:
  en la arquitectura FreeRTOS de §8 la tarea de UI corre normalmente durante las
  esperas de conversión. No es CPU ocupada, es la tarea de control esperando.

### 6.2 Ruido medido de la cadena PT100

Con la sonda al aire, 60 muestras (comando `ruido`):

| | Media | Desv. típica |
| :--- | ---: | ---: |
| Cruda | 26.651 °C | 0.039 °C |
| Filtrada (mediana + IIR α=0.20) | 26.652 °C | 0.010 °C |

Reducción de ruido ~3.8×. La medición está ~30× por debajo de la tolerancia
clase A (±0.3 °C): **el sensor no será el factor limitante del control.**
Repetir esta medida con el SSR conmutando 120 VAC cerca para verificar que el
ruido conducido no degrada la cifra.

Librerías: `Adafruit_MAX31865`, `QuickPID`, `sTune`, `U8g2` o
`Adafruit_SSD1306`, `Preferences`. Encoder por PCNT del ESP32 (filtra rebotes por
hardware) o por interrupción con debounce.

---

### 6.3 Etapa de salida — VERIFICADA CON CARGA AC

PWM lento time-proportional, ventana de 2 s, implementado en `02_pwm_lento/`.

**Arquitectura FreeRTOS (adelantada desde §8 por necesidad).** La lectura del
sensor bloquea 225 ms; dentro del mismo bucle que genera el PWM eso desfasaría la
ventana hasta un 11 % de duty. Separación adoptada:

| Tarea | Prioridad | Periodo | Puede bloquearse |
| :--- | :---: | :--- | :--- |
| `tareaSalida` | 3 | 5 ms | **No** — solo conmuta el GPIO |
| `tareaSensor` | 2 | 1 s | Sí — 225 ms de conversiones |
| `loop()` (consola) | 1 | 20 ms | Sí |

**Exactitud medida** (contabilidad interna de tiempo ON por ventana, sin carga):

| Comandado | Real | Error |
| ---: | ---: | ---: |
| 1 % | 1.00 % | 0.00 |
| 2 % | 2.00 % | 0.00 |
| 5 % | 5.00 % | 0.00 |
| 25 / 50 / 75 % | idem | 0.00 |
| 99 % | 99.00 % | 0.00 |
| 100 % | 100.00 % | 0.00 |

La lectura del sensor bloqueando 225 ms no perturba el PWM: la separación de
tareas funciona.

**Saturación de pulso mínimo.** A 60 Hz cada semiciclo dura 8.33 ms; un SSR
zero-cross no puede reproducir pulsos más cortos. `PULSO_MIN_MS = 20 ms` (~2.4
semiciclos): por debajo satura a 0 %, por encima de `ventana - 20 ms` satura a
100 %. Con ventana de 2 s eso da un rango útil de 1–99 % con resolución de 0.25 %
(la tarea corre cada 5 ms).

**Watchdog.** El core de Arduino ya inicializa el TWDT, así que hay que usar
`esp_task_wdt_reconfigure()`, no `esp_task_wdt_init()` (que solo imprime
"TWDT already initialized"). Timeout 8 s, `tareaSalida` registrada.
*Pendiente:* verificar que el WDT realmente dispara — hacerlo en el paso 7, no
antes, para no dejar código que cuelgue a propósito en un sketch que maneja carga.

**Verificación con carga AC real** (serie de prueba con bombilla incandescente,
~0.5 A, el SSR sustituyendo al puente de las puntas):

| Comandado | Real | Observación visual |
| ---: | ---: | :--- |
| 25 % | 25.00 % | Pulsos netos, proporcionales |
| 50 % | 50.00 % | Idem |
| 75 % | 75.00 % | Idem |
| 0 / 100 % | 0.00 / 100.00 % | Idem |

Montaje recomendado para pruebas: **serie de prueba**, no lámpara suelta. La
bombilla en serie limita la corriente por sí misma, así que un SSR en corto solo
la deja encendida — no hay evento peligroso — y sirve de testigo visual a la vez.
Usar bombilla de 60 W o más: el SSR-25 DA necesita corriente mínima de carga para
engancharse, y una de 15 W (125 mA) podría quedar en el límite y dar disparo
errático indistinguible de un problema de tensión de control.

**Interferencia del SSR sobre la PT100** (comando `emi`):

| Condición | Media | Desv. típica |
| :--- | ---: | ---: |
| SSR en reposo | 26.701 °C | 0.0261 °C |
| SSR conmutando al 50 % | 26.688 °C | 0.0261 °C |

Degradación **1.0×**: sin interferencia medible.

> ⚠️ **Este resultado NO es concluyente para el montaje final.** La bombilla
> consume ~0.5 A; la resistencia consumirá entre 5 y 14 A según se resuelva §1.1
> — de 10 a 28 veces más corriente conmutada. Además el cable de la PT100 aún no
> está tendido junto al de potencia dentro del gabinete. **Repetir `emi` con la
> carga real y el cableado definitivo** antes de dar por buena la inmunidad.

**Reset por apertura del puerto serie.** El circuito de auto-reset del DevKit se
dispara al activar DTR/RTS, que es lo que hace pyserial (y el monitor del IDE) al
abrir el puerto. Consecuencias:
- Verificado en la práctica que el reset deja la salida en LOW y arranca
  desarmado, como exige §7.2. La protección funciona.
- **En el paso 4 el script de registro debe abrir el puerto UNA sola vez y
  mantenerlo abierto durante toda la curva**, o se pierde el ensayo a mitad.
- En el gabinete final el ESP32 no estará conectado a un PC, así que en
  producción es irrelevante; durante el desarrollo no.

**Etiqueta engañosa a corregir en el paso 3.** Durante el primer segundo tras el
arranque la tarea de sensor aún no ha completado su primera lectura y el estado
muestra `nan C [FALLA]`. El comportamiento es correcto y seguro (`armar` se niega
sin lectura válida), pero *"sin medida todavía"* no es *"falla"* y en un panel de
operador esa diferencia importa. Añadir un tercer estado a la UI.

**Seguridad implementada en este sketch:**
- Salida arranca **DESARMADA**; requiere comando `armar` explícito.
- Hombre muerto: 10 min sin comandos → duty 0 y desarme.
- Falla del MAX31865, NaN o lectura fuera de 0–150 °C → desarme inmediato.

---

## 7. SEGURIDAD — REQUISITOS NO NEGOCIABLES

### 7.1 Capas de protección física
1. **Válvula de seguridad original de la olla** — protección primaria real contra
   sobrepresión. No retirar, no modificar, verificar libre antes de cada ciclo.
2. **Termostato bimetálico NC a 130 °C** en serie con la fase. El valor original
   de 135–140 °C es demasiado alto: a 140 °C la presión de saturación es ~2.6 bar
   manométricos (≈38 psi), muy por encima de lo que maneja una 75X.
3. **Fusible** dimensionado según §1.1.
4. **Pull-down de 10 kΩ** en la línea de control del SSR (§3.1-B).

### 7.2 Protecciones en firmware
- **Límite absoluto:** si T > 128 °C, cortar salida incondicionalmente e ir a
  FALLA, sin importar lo que diga el PID.
- **Detección de operación en seco:** el termostato del termowell **no** protege
  contra esto. Si el agua se evapora, la resistencia puede llegar a 400 °C
  mientras la pared de la olla aún lee valores moderados. Implementar: si la
  salida está al 100 % durante X minutos y la temperatura no ha subido al menos Y
  grados, cortar y alarmar. Calibrar X e Y tras resolver §1.1.
- **Fallas del sensor:** el MAX31865 reporta RTD abierto, corto y sobre/subtensión.
  Ante cualquier flag → salida a 0 % y FALLA. Ante lectura fuera del rango
  0–150 °C → igual tratamiento.
- **Watchdog** activo.
- **Estado inicial seguro:** al arrancar, salida en 0 % y máquina de estados en
  IDLE. Nunca calentando. `digitalWrite(SSR, LOW)` como primera línea de
  `setup()`, antes de cualquier otra inicialización.
- **Estado de FALLA es terminal:** solo se sale con reset manual del operador,
  nunca automáticamente.

---

## 8. INTERFAZ DE USUARIO

Un solo encoder con pulsador.

- **Pantalla principal:** estado del ciclo, T actual, setpoint, % de salida,
  F₀ acumulado, tiempo transcurrido.
- **Menú:** iniciar ciclo, parámetros (setpoint, F₀ objetivo, Kp/Ki/Kd),
  autotune, modo manual, historial del último ciclo.
- **Confirmaciones:** el checklist previo y el cierre de válvula requieren
  pulsación explícita.
- **Alarmas:** buzzer diferenciado para fin de ciclo (agradable) y falla
  (insistente).

Separar tareas con FreeRTOS: una tarea de lectura/control a ritmo fijo, otra de
UI. La tarea de control nunca debe bloquearse por el refresco de pantalla.

---

## 9. ORDEN DE DESARROLLO

1. Sketch de validación: leer PT100 y verificar configuración del MAX31865
   (Rref, 3 hilos, filtro 60 Hz). Contrastar con hielo (0 °C) y agua hirviendo.
2. Salida PWM lento probada con una lámpara incandescente, no con la resistencia.
3. Encoder + OLED, navegación de menús.
4. Caracterización de planta: escalón al 50 %, registrar curva, extraer L, τ, K.
5. PID + autotune con `sTune`.
6. Máquina de estados completa con F₀.
7. Capas de seguridad en firmware.
8. Montaje en gabinete final y ciclo de validación con sonda de penetración.

---

## 10. ESTADO DEL DESARROLLO

| Paso | Estado | Sketch |
| :--- | :--- | :--- |
| 0. Comunicación ESP32 ↔ PC | ✅ Verificado | `test_comunicacion/` |
| 1. Validación PT100 / MAX31865 | ✅ Cadena OK; calibración diferida (§10.1) | `01_validacion_pt100/` |
| 2. PWM lento con lámpara | ✅ Verificado con carga AC real | `02_pwm_lento/` |
| 3. Encoder + OLED | 🔄 Buzzer y pantalla OK; faltan 2 puntos (§10.2) | `03_ui_encoder_oled/` |
| 4. Caracterización de planta | ✅ Planta abierta y cerrada medidas (§10.12) | `04_caracterizacion/` |
| 5. PID | ✅ Rampa + relevo + precarga. Sobrepaso +1.06 °C (§10.13) | `04_caracterizacion/` |
| 6. Máquina de estados + F₀ | ✅ Ciclo completo validado (§10.14) | `04_caracterizacion/` |
| 7. Capas de seguridad | 🔄 Adelantadas al paso 4 (ver §10.5) | `04_caracterizacion/` |
| 8. Gabinete + validación | ⬜ Pendiente | — |

### Hardware verificado
- **Placa sustituida** (la original quedó fuera de servicio). La actual usa
  adaptador **CP210x** y aparece en **COM7**; la anterior era CH340 en COM12.
  Mismo pinout, mismo FQBN `esp32:esp32:esp32`, 115200 baudios.
  > El NVS con los parámetros guardados vive en la placa, no en el proyecto: al
  > cambiar de ESP32 se vuelve a los valores por defecto de `cargarParametros()`.
  > Reintroducir setpoint, F₀ y PID, y volver a "Guardar en NVS".
- `subir.ps1` **detecta el puerto automáticamente** (CH340/CP210x/CDC); usar
  `-Puerto COMx` solo si hay varios candidatos.
- Bus I2C operativo en SDA=21 / SCL=22, sin dispositivos conectados aún
- **OLED I2C en 0x3C** (el firmware la autodetecta en 0x3C/0x3D y reintenta)
- **Encoder KY-040** por PCNT y **buzzer** con etapa TIP122 (§3.3): operativos
- **MAX31865 + PT100 3 hilos**: operativo. Rref = 430 Ω correcto, sin flags de
  falla, 110.39 Ω / 26.68 °C a ambiente. Ruido y latencia medidos en §6.1 y §6.2.
  La sonda ya está instalada en el termowell del autoclave. Tras reparar el
  terminal `F+` (§10.9), la dispersión quedó en **0.03 °C** sin flags.

### 10.1 Calibración — diferida a comparación in situ
La validación de dos puntos con hielo y agua hirviendo **no se hará**: la sonda ya
está montada en el termowell y desmontarla no compensa. En su lugar, el Sr. Carlos
dispone de un **equipo de laboratorio** que se usará como patrón para comparar
contra la lectura del sistema durante un ciclo real.

Es mejor referencia que el baño de hielo: valida el conjunto sonda + termowell +
montaje a la temperatura de trabajo, no la sonda suelta. Programar la comparación
antes del paso 5 (PID + autotune), porque los parámetros del lazo se ajustan sobre
la lectura ya corregida.

Los comandos `hielo` y `agua` del sketch del paso 1 quedan disponibles por si
alguna vez se desmonta la sonda.

### Datos pendientes de confirmar
- **§1.1** — potencia real de la resistencia (medir con pinza amperimétrica)
- ~~Altitud del sitio~~ — **resuelto**: 1495 m, Medellín (ver §11.1)

---

## 10.2 Paso 3 — pendiente de verificación manual

`03_ui_encoder_oled/` está implementado, compila y arranca sin errores (el PCNT
inicializa correctamente: `ESP_ERROR_CHECK` habría abortado el arranque). Integra
las etapas ya validadas de los pasos 1 y 2.

**Cuatro puntos que solo se pueden comprobar mirando el equipo:**

| # | Comprobar | Corrección si falla |
| :---: | :--- | :--- |
| 1 | Imagen centrada, sin desplazamiento de ~2 px ni basura en el borde | El display es SH1106: poner `CONTROLADOR_SH1106` a 1 |
| 2 | Un click de detente = un elemento de menú | Ajustar `PASOS_POR_DETENTE` (ahora 4); comando `enc` cuenta detentes |
| 3 | Giro horario baja en el menú y sube el valor | Invertir las acciones de canal del PCNT |
| 4 | El buzzer suena (opción "Probar buzzer") | Buzzer activo no responde bien a `tone()`, o necesita transistor |

**Implementado en este paso:**
- Encoder en cuadratura x4 por **PCNT** con filtro antirrebote de hardware (1 µs),
  sin interrupciones que compitan con las tareas.
- Pulsador con distinción corta / larga (umbral 600 ms) para entrar y volver.
- Tercera tarea FreeRTOS `tareaUI` (prio 1, 40 ms) en el **core 0**; salida y
  sensor siguen en el core 1. El refresco de pantalla no puede desfasar el PWM.
- **Corregido lo detectado en el paso 2**: el estado del sensor ahora tiene tres
  valores (`SIN MEDIDA` / `OK` / `FALLA`) en vez de dos, y la falla muestra el
  motivo concreto.
- Persistencia en **NVS** vía `Preferences`: setpoint, F₀ objetivo, Kp/Ki/Kd y
  ventana de PWM sobreviven al reinicio.
- Modo manual de salida desde la pantalla, sujeto al mismo desarme por falla de
  sensor y al hombre muerto.

---

## 10.3 Alimentación — incidencia resuelta y lecciones

**Síntoma:** alimentando por el pin `5V` desde la fuente externa, el ESP32
arrancaba pero la OLED quedaba negra. Por USB funcionaba todo perfecto.

**Causa raíz: un pin GND con mal contacto.** Al pasar el cable a otro de los
cuatro GND de la placa, funcionó de inmediato. El contacto era suficiente para
que el ESP32 arrancara, pero no para sostener el retorno de corriente del
conjunto — de ahí que solo se cayera la pantalla.

**Medida durante el fallo:** pin `3V3` a **3.1 V** en vez de 3.3 V. El AMS1117
del DevKit necesita 1.0–1.3 V de caída, así que 3.1 V a la salida delata que a la
entrada no llegaban 5 V reales.

**Por qué se caía justo la pantalla y nada más.** El SSD1306 genera ~7.5 V para
el panel con una bomba de carga interna alimentada desde VDD. A 3.1 V el
controlador arranca y responde por I²C, pero la bomba no alcanza la tensión que
necesita el panel: hay comunicación y no hay imagen. El MAX31865 aguantaba porque
su VDD mínimo es 3.0 V.

**Lección para el gabinete final:** un GND flojo no se manifiesta como un fallo
limpio sino como el periférico más exigente cayéndose. Al montar en el gabinete,
usar terminales engarzados o soldados en vez de dupont, y **medir el `3V3` bajo
carga** como comprobación de rutina: por debajo de 3.2 V hay que corregir antes
de seguir.

### 10.3.1 Sobre alimentar periféricos a 5 V

Se evaluó mover módulos al rail de 5 V para descargar el de 3.3 V. **Los GPIO del
ESP32 no toleran 5 V** (máximo absoluto VDD + 0.3 = 3.6 V), así que lo que decide
no es de dónde se alimenta el módulo sino a qué tensión quedan sus líneas de
señal:

| Módulo | ¿5 V? | Motivo |
| :--- | :---: | :--- |
| OLED HS-F19-L | Sí, con reserva | Admite 5 V, pero **verificar antes que los pull-ups de SDA/SCL referencien 3.3 V y no 5 V** midiendo con el módulo alimentado y desconectado del ESP32 |
| MAX31865 | **No** | VDD del chip 3.0–3.6 V. Los módulos genéricos no llevan regulador ni adaptación de niveles: 5 V lo destruyen. Y su `SDO` pasaría a 5 V sobre GPIO19 |
| KY-040 | **No** | Son interruptores con pull-ups a VCC: a 5 V sus tres salidas quedan a 5 V en reposo, directas a GPIO26/27/14 |

Resuelto el GND, nada de esto hace falta: todo funciona a 3.3 V como estaba
diseñado.

---

## 10.4 Conflicto: detección de operación en seco vs. meseta de ebullición

Al implementar §7.2 apareció una interacción que el diseño original no contempla.

**El detector ingenuo** ("si la salida está al 100 % durante X minutos y la
temperatura no ha subido Y grados, cortar") **da falso positivo durante la purga
de aire.** En la Fase 1 (§5) se calienta al 100 % con la válvula abierta y la
temperatura se queda clavada en el punto de ebullición — 94.9 °C en Medellín —
durante 7 a 10 minutos mientras el agua absorbe calor latente. Eso es
exactamente el patrón que el detector busca, y es funcionamiento normal.

**La firma real de la operación en seco es la contraria:** mientras queda agua,
la temperatura se estanca en la ebullición; **cuando el agua se acaba, sube
rápido** porque desaparecen la masa térmica y el calor latente.

**Diseño correcto: el detector debe conocer la fase del ciclo.**

| Fase | Detección de seco |
| :--- | :--- |
| Purga (meseta de ebullición) | **Desactivada** o con umbral muy largo — el estancamiento es normal |
| Rampa y meseta presurizada | **Activa** — aquí sí debe subir |
| Cualquier fase | Límite absoluto de 128 °C **siempre activo**, es el respaldo real |

Además, en la meseta presurizada la temperatura supera la ebullición de forma
legítima, así que una subida rápida por sí sola tampoco basta para declarar seco:
hay que combinarla con la fase y con el duty aplicado.

**Provisional para el paso 4:** umbral conservador (duty ≥ 80 % durante 10 min
con ΔT < 5 °C) y el límite absoluto de 128 °C como respaldo. Recalibrar tras
resolver §1.1, e implementar la versión consciente de la fase en el paso 6.

---

## 10.5 Paso 4 — listo, bloqueado por hardware de seguridad

**Corrección al orden de §9.** El orden pone las capas de seguridad en el paso 7,
pero **el paso 4 es el primero que energiza la resistencia real**. Las
protecciones de §7.2 van incorporadas en `04_caracterizacion/`, no después.

**Implementado en el firmware:**

| Protección (§7.2) | Estado |
| :--- | :--- |
| `digitalWrite(SSR, LOW)` como primera línea de `setup()` | ✅ |
| Salida arranca desarmada, requiere `armar` | ✅ |
| Límite absoluto 128 °C → corte incondicional | ✅ |
| Falla de sensor (flag, NaN, fuera de 0–150 °C) → corte | ✅ |
| Detección de operación en seco | ✅ provisional, ver §10.4 |
| Watchdog de 8 s sobre la tarea de salida | ✅ |
| **FALLA terminal**, solo se sale con `reset` manual | ✅ |
| Aviso audible al entrar en falla (5 tonos graves) | ✅ |

**Herramienta de registro: `herramientas/registrar.py`**

Abre el puerto **una sola vez** y lo mantiene abierto durante todo el ensayo,
que es obligatorio: reconectarse resetea el ESP32 y pierde la curva. Registra
CSV en `ensayos/`, y al terminar extrae los parámetros FOPDT por el método de la
tangente.

```
python herramientas/registrar.py --duty 50 --minutos 45
python herramientas/registrar.py --analizar ensayos/escalon_50pct_XXXX.csv
```

**Analizador validado contra una curva sintética de parámetros conocidos:**

| Parámetro | Real | Extraído | Error |
| :--- | ---: | ---: | ---: |
| L (tiempo muerto) | 90.0 s | 89.9 s | 0.1 % |
| τ (constante de tiempo) | 900.0 s | 896.1 s | 0.4 % |
| K (ganancia) | 0.600 C/% | 0.588 C/% | 2 % |

El 2 % de K es correcto: a 3600 s la curva lleva 3.9 τ y le falta un 2 % para
asentar. Con ensayo más largo converge.

Además del análisis, propone un punto de partida PI por Ziegler-Nichols en lazo
abierto, y **avisa si la curva atraviesa el punto de ebullición** — porque la
dinámica antes y después del cambio de fase es distinta y esos parámetros
mezclarían dos plantas (§6).

### BLOQUEANTES antes de ejecutar el paso 4

No es código, es hardware de §7.1:

- [ ] **Termostato bimetálico NC de 130 °C** en serie con la fase — único
      recurso si el SSR falla en corto; ningún firmware lo sustituye
- [ ] **Fusible** en la fase, dimensionado según §1.1
- [ ] **SSR sobre disipador** con pasta térmica
- [ ] **Válvula de seguridad** de la olla verificada y libre
- [ ] Nivel de agua correcto, resistencia sumergida
- [ ] Operador presente durante todo el ensayo

Y el ensayo es la ocasión para **resolver §1.1 con pinza amperimétrica**: de esa
medida salen el dimensionado del fusible, si el disipador necesita aletas, y los
umbrales definitivos de detección de operación en seco.

---

## 10.6 Montaje del termostato bimetálico sin perforar la olla

**Descartado: montarlo en el cajón de la electrónica.** Mediría el aire del
cajón, que como mucho llega a 40-50 °C, así que nunca cortaría. Sería protección
aparente, más peligrosa que no tenerla. Su único trabajo (§7.1) es cortar
mecánicamente cuando el SSR falla en corto y el firmware ya no manda nada: tiene
que sentir la olla.

**No hace falta perforar.** La olla es aluminio fundido (~200 W/m·K) y la pared
exterior está prácticamente a la temperatura del vapor interior:

| | |
| :--- | ---: |
| Espesor de pared | 5–8 mm |
| Flujo de calor al ambiente (121 °C int., 25 °C amb.) | ~1.100 W/m² |
| **Salto térmico a través de la pared** | **~0.05 °C** |

Montarlo pegado por fuera es térmicamente equivalente a meterlo dentro.

**Montaje, por orden de importancia:**

1. **Aislarlo por fuera** con lana cerámica o fibra de vidrio. Es lo que más se
   olvida y lo que más error mete: sin aislante el termostato pierde calor al
   aire ambiente y se queda **10-20 °C por debajo** de la pared que toca.
2. **Pasta térmica** >150 °C entre su cara plana y la pared.
3. **Presión mecánica constante**: abrazadera inox o fleje con resorte. Nada de
   cinta ni adhesivo, a 130 °C se despega.
4. **Ubicación**: pared lateral, **por encima del nivel del agua**, en la zona de
   vapor — que es donde la temperatura se corresponde con la presión.

**Cableado del termostato:** cable siliconado o con camisa de fibra de vidrio.
El PVC normal se reblandece por encima de 105 °C y este va sobre una superficie a
121 °C con picos de 130. En serie con la **fase**, nunca con el neutro.

### 10.6.1 Verificación obligatoria durante el paso 4

La física dice que el error es despreciable, pero con presión de por medio hay
que medirlo. Durante el ensayo de caracterización, con la PT100 registrando el
interior, medir con termómetro infrarrojo:

- La superficie de la olla justo al lado del termostato
- El cuerpo del termostato

| Diferencia respecto a la PT100 | Acción |
| :--- | :--- |
| < 5 °C | El modelo de 130 °C es correcto |
| 5–10 °C | Mejorar el aislamiento y repetir |
| > 10 °C | Cortaría demasiado tarde. Bajar a un termostato de 110–115 °C |

---

## 10.7 Validez del sensor: por la lectura, no por el registro de fallas

**Problema encontrado.** Tras instalar la PT100 en el autoclave con su cable
definitivo, el MAX31865 empezó a levantar flags `REFIN-`/`RTDIN-` de forma
intermitente. La primera versión del firmware del paso 4 latchaba un estado de
FALLA terminal ante cualquier flag, así que el equipo arrancaba directamente
bloqueado.

**Dos causas, una de firmware y una de hardware.**

### Firmware — respuesta desproporcionada

| Defecto | Corrección |
| :--- | :--- |
| Actuaba desde la primera lectura | **4 s de gracia** al arrancar |
| Un flag suelto latchaba estado terminal | **Antirrebote**: 3 lecturas seguidas (2 para sobretemperatura) |
| Latchaba aunque no hubiera nada energizado | **Latcha solo si la salida está armada o hay ensayo** |
| Mensaje genérico | **Decodifica el flag** concreto y lo muestra en pantalla |

**Cambio de criterio: la validez del sensor se decide por la LECTURA.** El ciclo
automático de detección de fallas del MAX31865 sólo espera 1 ms a que asiente la
corriente de bias (así lo implementa la librería Adafruit); con el cable largo de
la instalación esa espera se queda corta y produce falsos positivos.

No debilita la protección, porque los modos de fallo reales se ven en la lectura:

| Fallo físico | Cómo se detecta |
| :--- | :--- |
| RTD abierta | `raw` saturado |
| RTD en corto | `raw` ≈ 0 |
| FORCE− abierto | sin corriente → `raw` ≈ 0 |
| Contacto intermitente | salto > 5 °C entre muestras de 1 s (imposible en esta planta) |
| Deriva de calibración | **No se detecta** — tampoco por el registro de fallas. Para eso está §10.1 |

El registro de fallas se conserva como diagnóstico, y sigue invalidando si
persiste 5 lecturas seguidas. Comando `diag` para verlo decodificado.

### Hardware — contacto marginal en F− / RTD−

Medición sobre 51 s comparada con la del banco:

| | Banco (§6.2) | Tras instalar |
| :--- | ---: | ---: |
| Dispersión de la lectura | 0.039 °C | **0.58 °C** (0.223 Ω) |
| Lecturas con flag | 0 % | **30 %** |
| Familia del flag | — | **siempre** RTDIN− / REFIN− |
| Flag OVUV (alimentación) | — | **nunca** |

Tres indicios convergentes: el flag nunca es aleatorio, nunca aparece OVUV (lo
que **descarta la alimentación**), y el ruido se multiplicó por 15.

**Diagnóstico: contacto marginal en el lado F− / RTD− del bloque izquierdo.**
Pasa corriente suficiente para que la lectura salga aproximadamente bien, pero la
detección de fallas lo caza. Acción: reapretar tornillos, verificar el puente
F− ↔ RTD− (~0 Ω), y revisar el engarce del hilo suelto de la PT100.

> Aunque el firmware ya lo tolera, **hay que arreglarlo**: 0.58 °C de ruido
> desperdicia la precisión de ±0.3 °C de una PT100 clase A, en un proceso donde
> la letalidad depende exponencialmente de la temperatura.

## 10.8 Subida de firmware: dos tropiezos y sus remedios

**`Wrong boot mode detected (0x13)`.** El ESP32 no entra en modo descarga.
Remedio manual, deja el chip en modo descarga de forma estable:

1. Mantener pulsado **BOOT**
2. Con BOOT pulsado, pulsar y soltar **EN**
3. Soltar **BOOT**

Causa probable: **GPIO5 es pin de strapping** (§3.1-C) y es el CS del MAX31865.
Si se vuelve recurrente, mover el CS **de GPIO5 a GPIO32** — un cable y una
constante. GPIO32 está libre; GPIO33 lo ocupa el buzzer.

También conviene **desconectar la fuente externa mientras se graba** y dejar solo
el USB: dos fuentes en el mismo rail alteran el arranque.

**`The chip stopped responding` a mitad de escritura.** La velocidad por defecto
de 921600 baudios es agresiva. Se resuelve bajando a 115200:

```
arduino-cli upload -p COM7 --fqbn "esp32:esp32:esp32:UploadSpeed=115200" <sketch>
```

Tarda 21 s en vez de 4, pero no falla. **Usar esta forma por defecto** con el
montaje definitivo.

---

## 10.9 Avería resuelta: el terminal F+ del MAX31865

**Síntoma final:** `raw = 0` constante. Ninguna sonda daba lectura, ni siquiera
una resistencia fija de 130 Ω conectada directamente en la bornera.

**Lo que NO era**, descartado con datos:

| Hipótesis | Cómo se descartó |
| :--- | :--- |
| Sonda dañada | Una PT100 nueva dio el mismo `raw = 0` |
| Cableado de la sonda | Una resistencia fija de 130 Ω en la bornera también dio 0 |
| Chip muerto | Volcado de registros: escritura 0xD1 → lectura 0xD1, SPI perfecto |
| Configuración | Config = 0x90 = VBIAS activo + modo 3 hilos, correcto |
| Alimentación | El flag OVUV no apareció **nunca** en todo el diagnóstico |
| Contacto flojo | 170 s moviendo cables sin un solo parpadeo en la lectura |

**Lo que era:** el terminal **`F+` aislado del resto del circuito**, detectado
midiendo continuidad terminal por terminal. `RTD+`, `RTD−` y `F−` conectados
entre sí; `F+` sin continuidad con nada.

**Por qué eso lo explica todo.** Los cuatro terminales no hacen el mismo trabajo:

| Terminal | Función |
| :--- | :--- |
| **`F+`** | **FORCE+ — por aquí SALE la corriente de excitación** |
| `RTD+` | Solo mide tensión, no inyecta corriente |
| `RTD−` | Solo mide |
| `F−` | FORCE− — retorno de la corriente |

Sin `F+` no circula corriente, no hay caída de tensión, y el ADC lee cero
independientemente de lo que haya conectado en los otros tres.

**Resultado tras la reparación:**

| | Banco original (§6.2) | Durante la avería | Reparado |
| :--- | ---: | ---: | ---: |
| Dispersión | 0.039 °C | 0.58 °C | **0.03 °C** |
| Lecturas con flag | 0 % | 30 % | **0 % (12/12)** |

Queda **más estable que la medida original de banco**.

### 10.9.1 Lo que revela hacia atrás — y qué vigilar

`F+` era la causa de toda la secuencia, no solo del corte final:

1. **Flags intermitentes** (§10.7) → contacto marginal que conducía a ratos
2. **Ruido ×15** (0.039 → 0.58 °C) → el mismo contacto degradándose
3. **`raw = 0`** → el contacto terminó de abrirse

> **Regla para el mantenimiento:** un contacto marginal en `F+` no falla de
> golpe. Se degrada primero como **ruido en la lectura y flags intermitentes**.
> Si vuelve a subir la dispersión por encima de ~0.1 °C, revisar `F+` antes que
> ninguna otra cosa.

### 10.9.2 El módulo es un CJMCU-865, no un Adafruit

Confirmado en la serigrafía trasera. **Su distribución de puentes de 2/3/4 hilos
no coincide con la de Adafruit**, así que las instrucciones de Adafruit
(«cortar la pista fina, soldar la gota del lado derecho») no se aplican
directamente. Si en el futuro se sustituye el módulo, buscar la documentación del
modelo concreto antes de modificar nada.

Etiquetas de los terminales, legibles en la **cara trasera**: `F+ RTD+ RTD− F−`.

---

## 10.10 El manómetro de la olla induce a error a esta altitud

El 75X trae un manómetro combinado con la temperatura de saturación marcada
junto a cada presión: 15 psi ↔ 121 °C, 20 psi ↔ 127 °C, etc. **Esa escala está
calculada para el nivel del mar.**

A 1.495 m la presión atmosférica es 84.6 kPa en vez de 101.3, así que la misma
presión manométrica corresponde a una temperatura **menor**:

| Manómetro | **Temperatura real en Medellín** | La escala dice | Error |
| ---: | ---: | ---: | ---: |
| 5 psi | 104.7 °C | ~109 °C | −4.3 |
| 10 psi | 112.2 °C | ~115 °C | −2.8 |
| **15 psi** | **118.5 °C** | **121 °C** | **−2.5** |
| **17.5 psi** | **121.1 °C** | ~124 °C | — |
| 20 psi | 123.8 °C | ~127 °C | −3.2 |

> ⚠️ El error va en la dirección peligrosa: **quien lea la escala creerá estar
> más caliente de lo que está.** Confiando en la marca de 121 °C se esterilizaría
> a 118.5 °C.

**Acción: etiqueta física junto al manómetro.** El operador que use el equipo
dentro de seis meses no tendrá este historial:

```
  ATENCION - MEDELLIN 1495 m
  La escala de °C de este manometro
  es para nivel del mar.
  Para 121 C reales:  17.5 psi
  A 15 psi solo hay:  118.5 C
```

**Refuerza la decisión de §5 Fase 3.** Con temporizador fijo, esos 2.5 °C de
menos suponen la mitad de letalidad en el mismo tiempo. Con acumulación de F₀ el
ciclo simplemente dura más y llega igual: **el sistema es inmune a este error.**

**Verificación cruzada:** en la meseta, con la PT100 en 121 °C el manómetro debe
marcar ~17.5 psi. Es una comprobación de cordura gratuita entre dos instrumentos
independientes.

## 10.11 Calibración contra patrón — estado

**Patrón disponible: Hanna Checktemp**, certificado EN 13485, resolución 0.1 °C,
exactitud ±0.3 °C en rango medio.

**Primera medida (2026-09-09, a 26 °C ambiente):** patrón 26.0 °C, sistema
26.6 °C → desviación **+0.6 °C**. Incertidumbre combinada ≈ 0.36 °C, así que la
desviación es real. Aplicado `offsetC = −0.60` (firmware v0.5.1, guardado en NVS).

**Limitación:** calibrado a 26 °C para un proceso que trabaja a 121 °C. Traducido
a resistencia son 0.24 Ω; si es offset fijo se mantiene en 0.60 °C a 121 °C, si
es error de ganancia crece a 0.83 °C. Solo 0.2 °C de diferencia entre ambos
casos, así que la corrección es razonable — pero hay que verificarla.

**No se puede verificar a 121 °C con este instrumento:** el Checktemp es una
sonda de penetración y la olla debe estar sellada para llegar a 121 °C. No hay
puerto por donde introducirla y el termowell lo ocupa la PT100.

**Plan alternativo, mejor:** calibrar en la **meseta de ebullición con la tapa
abierta**, donde hay **dos referencias independientes**:

| Referencia | Valor | Incertidumbre |
| :--- | :--- | :--- |
| Checktemp sumergido | lo que marque | ±0.3 °C |
| Punto de ebullición (§11.1) | 94.94 °C | ±0.4 °C (varía con la presión del día) |

Si ambas coinciden, la calibración es sólida. Y son 95 °C en vez de 26: la
extrapolación hasta 121 °C pasa de 95 grados a 26.

---

## 10.12 Caracterización de la planta — dos plantas distintas

Dos ensayos, y el resultado más importante es que **la olla abierta y la cerrada
son plantas muy diferentes**.

| Parámetro | Olla abierta (escalón 50 %) | **Olla cerrada (PID 110 °C)** |
| :--- | ---: | ---: |
| Pérdidas U | 19.1 W/K | **6.29 W/K** |
| Ganancia K | 0.878 °C/% | **2.67 °C/%** |
| Constante τ | 1176 s (20 min) | **~5030 s (84 min)** |
| Tiempo muerto L | 136 s | ~136 s |
| Duty de régimen a 110 °C | — | **32 %** |
| Duty de régimen a 121 °C | — | **~37 %** |

**La evaporación se llevaba dos tercios de las pérdidas.** Al sellar la olla
desaparece esa vía: la ganancia se triplica y la constante de tiempo se
cuadruplica.

> Los parámetros de la olla abierta **no sirven** para sintonizar el lazo de la
> meseta. Es exactamente lo que advierte §6 al pedir el autotune en la zona de
> 110-121 °C y no desde temperatura ambiente.

**Cómo se midió U con la olla cerrada:** en el pico del sobrepaso dT/dt = 0, así
que la salida iguala exactamente a las pérdidas. Con 33 % de duty a 113.1 °C:
`U = 0.33 × 1680 / (113.1 − 25) = 6.29 W/K`. El valor coincidió con la
estimación hecha a mitad de la subida, así que el modelo es consistente.

**Verificación de la calibración:** en la meseta de ebullición la PT100 marcó
**95.2 °C** contra los **94.94 °C** teóricos para 1495 m. Diferencia de 0.3 °C,
dentro de la incertidumbre de la propia referencia (±0.4 °C por la presión
atmosférica del día). **El offset de −0.60 °C queda confirmado a 95 °C**, mucho
más cerca de los 121 de trabajo que los 26 °C donde se midió.

### 10.12.1 El sobrepaso y su causa real

Con Kp = 2.5 y Ki = 0.006, arrancando el integrador en cero:

| | |
| :--- | ---: |
| Tiempo de 95 a 110 °C | 29 min |
| **Pico** | **113.13 °C** |
| **Sobrepaso** | **+3.13 °C** |
| Integrador en el pico | 49 (el régimen pide 32) |

**La causa no es un Ki mal elegido.** En el setpoint el término proporcional vale
cero, así que **toda la potencia de mantenimiento tiene que salir del
integrador**. Partiendo de cero, el integrador debe recorrer ese camino entero, y
en una planta con τ de 84 min se pasa de largo mucho antes de que la temperatura
responda. Bajar Ki no lo arregla: con Ki = 0.002 tardaba 38 minutos en llegar.
No hay valor de Ki que resuelva las dos cosas a la vez.

Extrapolado a un setpoint de 121 °C, ese mismo sobrepaso llevaría a **~124 °C**,
demasiado cerca del límite de 128 y del termostato de 130.

### 10.12.2 Solución: precargar el integrador (v0.6.0)

Arrancar el lazo con el integrador ya en el duty de régimen estimado:

```
duty_regimen = DUTY_POR_GRADO × (setpoint − 25 °C)
DUTY_POR_GRADO = U / (P/100) = 6.29 / 16.8 = 0.374 %/°C

   110 °C → 32 %      121 °C → 36 %
```

El lazo empieza equilibrado y solo corrige la diferencia. Es la transferencia sin
salto de manual a automático de cualquier controlador industrial, y encaja con
§5: la rampa lleva la olla a 105-110 °C y **el PID entra ya cargado**.

**Pendiente de verificar** en el próximo ensayo: sobrepaso esperado < 1 °C.

### 10.12.3 El PID no debe hacer la purga

Durante la meseta de ebullición la temperatura se queda clavada 7-10 minutos con
un error grande y constante. La salida **no está saturada** (ronda el 60-90 %),
así que el anti-windup convencional no interviene y **el integrador se carga
igual** aunque la planta no pueda responder por el cambio de fase.

Se detectó en marcha: el duty bajaba de 95 a 88 % durante la meseta, señal del
integrador acumulando.

**Remedio aplicado:** abortar el PID al terminar la purga, cerrar la válvula y
relanzarlo, para que el integrador arranque limpio. **Para el paso 6 esto debe
ser automático:** el integrador se reinicia (y se precarga) en cada transición de
fase, no solo al arrancar.

---

## 10.13 Ciclo a 121 °C con rampa + relevo — RESULTADO

Firmware v0.7.1. Arquitectura de §5 implementada: **rampa al 100 % en lazo
abierto hasta setpoint − 10 °C, relevo al PID con el integrador precargado.**

| | Ensayo del 09-09 (PID desde 95 °C) | **Ciclo con rampa + relevo** |
| :--- | ---: | ---: |
| Sobrepaso | +3.13 °C | **+1.06 °C** |
| Integrador al llegar | 49 (régimen pedía 32) | **40 (régimen pide 36)** |
| Forma de la aproximación | brusca | monótona, sin oscilar |
| Estabilidad en meseta | — | **sd 0.012 °C, banda 0.041 °C** |
| F₀ total de la corrida | — | **46.2 min** (objetivo 20) |

**El relevo funcionó al decimal.** A 111.05 °C la salida pasó de 100 % a 76 %, que
es exactamente `Kp × error + precarga = 4.0 × 9.95 + 35.9 = 75.7`.

### 10.13.1 Verificación cruzada con el manómetro

Con la PT100 en **121.67 °C**, el manómetro marcó **18 psi**. La presión de
saturación a esa temperatura es 208.0 kPa absolutos; restando la atmosférica de
Medellín (84.6 kPa) salen **17.9 psi**.

**Dos instrumentos independientes coincidiendo dentro de 0.1 psi.** Valida a la
vez la cadena de medición, el offset de calibración de −0.60 °C y el cálculo de
altitud de §11.1.

Y confirma en vivo el error del manómetro: **su escala diría ~124 °C a 18 psi**,
cuando la temperatura real eran 121.67. Los 2.5 °C previstos en §10.10.

### 10.13.2 El residuo de +1 °C: qué es y por qué no preocupa

La meseta se estabilizó en **122.04 °C** en vez de 121.0, con una precisión
extraordinaria (sd 0.012 °C). **No es inestabilidad, es caída proporcional.**

Con Kp = 4 y el integrador en 39.2 cuando el régimen pide ~34.7, el término
proporcional aporta la corrección: `P = 4 × (−1.04) = −4.16`. Ese punto de
equilibrio es exactamente el observado.

El integrador **sí está corrigiendo**, pero a 0.062 puntos por minuto (Ki=0.001):
necesitaría ~50 minutos más para llegar a cero. La corrida terminó antes.

> **No es un defecto que bloquee nada.** A 122 °C el F₀ acumula a 1.14 min/min en
> lugar de 1.0: el error va en la dirección **segura** — más letalidad, no menos.
> Y 122 está a 6 °C del límite del firmware.

**U medida hoy en régimen:** `U = 0.35 × 1680 / (122.04 − 25) = 6.06 W/K`,
ligeramente por debajo de los 6.29 usados en la precarga. De ahí que la precarga
quedara ~1.3 puntos alta.

### 10.13.3 Mejora disponible, no bloqueante

Para afinar más, cuando haya tiempo:

1. **Actualizar U a 6.06 W/K** → precarga de 34.6 % en vez de 35.9 %
2. **Banda de integración**: acumular solo cuando `|error| < 2 °C`. Durante la
   aproximación el integrador acumuló ~4.5 puntos de más; congelarlo lejos del
   setpoint elimina esa fuente de sobrepaso sin tocar nada más
3. Con esas dos, **Ki puede subir a ~0.005** sin riesgo, porque solo actúa cerca
   del objetivo y no tiene que recorrer distancia

Predicción con las tres: sobrepaso < 0.3 °C y corrección del residuo en minutos.

---

## 10.14 CICLO COMPLETO VALIDADO — firmware v1.1.1

Primer ciclo de esterilización de principio a fin, con toda la secuencia de §5.
**Todas las fases funcionaron, y ninguna necesitó intervención por consola.**

| Fase | Comportamiento observado |
| :--- | :--- |
| CALENTANDO | 100 % en lazo abierto, válvula abierta |
| → PURGA | transición automática a **94.14 °C** (ebullición − 1) |
| PURGA | 7 min de cuenta atrás. La T subió a ~97 °C: **~1 psi de contrapresión**, normal con vapor saliendo |
| → CERRAR | aviso acústico al cumplirse los 7 min |
| CERRAR → RAMPA | **detectado solo a 99.57 °C** (+2 °C sobre los 97.08 de referencia) |
| RAMPA | 100 % hasta 111 °C, a 1.54 °C/min |
| → MESETA | relevo a **111.17 °C con salida 74 %** = `4.0 × 9.83 + 34.6` exacto |
| MESETA | convergencia asintótica, **sin sobrepaso** |
| → ENFRIANDO | F₀ alcanzó **20.00** y la salida se cortó sola |

**Resultado: pico 121.32 °C, sobrepaso +0.29 °C. Duración total 56 min.**

### 10.14.1 Progresión del sobrepaso — cada corrección se ve

| Corrida | Configuración | Sobrepaso |
| :--- | :--- | ---: |
| 1ª | PID desde 95 °C, integrador desde cero | **+3.13 °C** |
| 2ª | Rampa + relevo + precarga (U = 6.29) | **+1.06 °C** |
| 3ª | Precarga corregida (U = 6.06) + banda de integración | **+0.29 °C** |

Las tres causas eran distintas y cada arreglo atacó una:

1. **Rampa en lazo abierto + relevo cerca del setpoint** → el PID nunca satura,
   así que el integrador no se engancha al borde de saturación
2. **Precarga del integrador** al duty de régimen → no tiene que recorrer 35
   puntos desde cero mientras la planta, lenta, no responde
3. **Banda de integración** de ±2 °C → durante la aproximación el integrador
   está congelado; solo pule el último tramo

### 10.14.2 F₀ en funcionamiento

El ciclo **terminó sin haber estado nunca por encima de 121.32 °C**, y la
acumulación empezó mucho antes de llegar a la consigna:

| Momento | T | F₀ acumulado | Ritmo |
| :--- | ---: | ---: | ---: |
| Relevo del PID | 111.2 °C | 0.00 | — |
| Media subida | 117.4 °C | 2.45 | 0.39 min/min |
| Cerca de consigna | 119.7 °C | 7.51 | 0.72 min/min |
| En consigna | 121.0 °C | 16.5 | 0.98 min/min |
| Fin | 121.3 °C | **20.00** | 1.05 min/min |

**Casi 8 minutos de letalidad se acumularon antes de alcanzar la consigna.** Con
un temporizador fijo ese tiempo no habría contado, y el ciclo habría durado más
sin necesidad.

### 10.14.3 Avisos al operador implementados

- **Purga**: aviso acústico al llegar a la ebullición y al cumplirse los 7 min
- **Cierre de válvula**: pantalla parpadeante y tonos insistentes. **La
  confirmación la da la física**, no un botón: se detecta que la olla está
  sellada cuando la temperatura sube 2 °C. Un botón se puede pulsar sin haber
  cerrado nada
- **Reaviso** cada 5 min si la temperatura no sube (válvula sigue abierta)
- **Purga interrumpida**: si la T cae por debajo de la ebullición, vuelve a
  calentar y **reinicia el conteo**. Una purga a medias no barre el aire
- **Despresurización**: aviso al bajar de **90 °C** (no 100: a 1495 m eso serían
  todavía 2.4 psi), con reaviso cada minuto. El mensaje dice **"verifique el
  manómetro en cero"**, no "puede abrir": la temperatura es una inferencia, el
  manómetro es la medida directa

---

## 10.15 Falsos positivos de marcha en seco con carga real (v1.2.0)

**Síntoma:** con sustrato dentro, el ciclo abortaba repetidamente con
`posible marcha en seco: +4.9 C/10min`.

**Dos defectos, uno de calibración y uno de honestidad del mensaje.**

### El mensaje mentía

`SECO_VENTANA_MS` se había recalibrado a 5 minutos en la v0.4.3, pero el literal
del `snprintf` seguía diciendo `/10min`. El equipo reportaba una ventana que no
era la que usaba. Corregido: ahora el mensaje construye la ventana real desde la
constante, así que no puede volver a desincronizarse.

### El umbral estaba calibrado con la olla vacía

4.9 °C en 5 minutos son **0.98 °C/min**. Invirtiendo el balance térmico:

```
C = 1165 W / (0.98/60 K/s) = 71.300 J/K
    - olla y agua (45.000) = ~26.000 J/K de carga  ≈ 6 kg de sustrato húmedo
```

Es decir: **una rampa perfectamente sana, que falló por 0.1 °C.** El umbral de
1.0 °C/min se fijó con la olla vacía y no deja sitio a la carga real.

| | Antes | Ahora |
| :--- | :--- | :--- |
| Ventana | 5 min | **10 min** |
| Umbral | +5.0 °C | **+2.0 °C** |
| Tasa mínima exigida | 1.00 °C/min | **0.20 °C/min** |

Con la carga más pesada plausible la rampa sigue dando más de 5 °C en 10 min, y
ante una falta real de calor la temperatura **baja** — delta negativo, que es
inequívoco.

> Un detector que da falsos positivos acaba puenteado, y entonces no protege de
> nada. Aflojarlo hasta que solo dispare ante lo inequívoco lo hace más útil, no
> menos seguro.

### 10.15.1 Detector rápido de purga — el que de verdad sirve

Aprovechando que la máquina de estados ya sabe cuándo la válvula está abierta
(§5 Fases 1 y 2), se añade una comprobación que **no es heurística**:

> Con la válvula abierta, la temperatura **no puede** superar la ebullición
> mientras quede agua: el calor se va en vaporizar. Si sube claramente por
> encima, el agua se acabó.

`SECO_SOBRE_EBU_C = 8.0` → dispara por encima de **103 °C** durante
`F_CALENTANDO` y `F_PURGA`. Medido en purga real: ~97 °C, o sea 2 °C de
contrapresión por el vapor saliendo. Margen de 6 °C.

**Actúa en segundos en lugar de minutos**, y no puede dar falso positivo porque
no mide un ritmo: comprueba una imposibilidad termodinámica.

No se aplica en `F_CERRAR_VALVULA`, donde superar la ebullición es justamente lo
que se espera al sellar la olla.

Esto cierra lo que §10.4 dejó planteado: el detector consciente de la fase.

---

## 10.16 Práctica real del laboratorio: capas de 20 cm con cinta indicadora

**Dato aportado por el laboratorio (2026-09-30):** no esterilizan en capas de
5 cm como plantea §5.1, sino en **capas de 20 cm**, colocando cinta indicadora
en el interior de la carga. La cinta vira, y no han tenido incidencias.

Esto no invalida el proceso, pero conviene entender qué demuestra y qué no.

### Qué prueba la cinta

Clasificación ISO 11140:

| Clase | Qué indica |
| :--- | :--- |
| **1 — Proceso** | **Solo exposición al proceso.** La cinta beige de rayas diagonales |
| 4 — Multivariable | Responde a 2+ variables críticas |
| 5 — Integrador | Tiempo, temperatura y vapor. Se correlaciona con indicador biológico |
| 6 — Emulador | Verifica un ciclo concreto |
| Biológico | Esporas de *G. stearothermophilus*. Patrón de referencia |

**La cinta de rayas es Clase 1: vira a ~121 °C sin componente de tiempo**, en
segundos. Su propósito declarado es distinguir un bulto procesado de uno sin
procesar. La norma dice explícitamente que **no es evidencia de esterilización**.

Que vire indica que ese punto **llegó a temperatura en algún momento**. No dice
cuánto tiempo la mantuvo, que es justo lo que determina la letalidad.

### Por qué el espesor importa

El calor llega al centro del lecho principalmente por conducción, que escala con
el **cuadrado** de la distancia. Con difusividad térmica de arena húmeda
(α ≈ 9×10⁻⁷ m²/s) y Fo ≈ 1.2 para equilibrar el centro:

| Espesor | Distancia al centro | Tiempo (solo conducción) |
| ---: | ---: | ---: |
| 5 cm | 2.5 cm | ~14 min |
| **20 cm** | **10 cm** | **~3.7 h** |

> ⚠️ Ese cálculo es el **peor caso**: supone conducción pura. Si el vapor
> penetra en los poros de la arena húmeda, condensa dentro y entrega calor
> mucho más rápido. Cuánto más rápido depende del empaque, la humedad y la
> geometría del recipiente — no se puede calcular, **se mide**.

### Lo que esto NO significa

- **No significa que el proceso actual falle.** Sin incidencias durante un
  tiempo prolongado es información real, aunque no sea prueba formal.
- **No obliga a cambiar a capas de 5 cm.**

### Cómo F₀ lo resuelve sin cambiar el proceso

El F₀ que calcula el equipo es el **de la cámara**, medido en el termowell. Si
el centro de la carga va con retraso, su F₀ es menor — pero la relación entre
ambos es aproximadamente constante para una carga dada.

**Una vez medida esa relación, basta con subir el F₀ objetivo.** Si el centro
alcanza F₀ = 12 cuando la cámara marca 20, se fija el objetivo en 33 y el centro
llega a 20. Es un parámetro del menú, y el laboratorio conserva sus capas de
20 cm.

### 10.16.1 Esto concreta el paso 8

La validación de penetración deja de ser genérica y pasa a tener un objetivo
numérico:

1. Segunda sonda enterrada en el **centro geométrico** de la carga habitual
2. Un ciclo normal registrando ambas temperaturas
3. Calcular F₀ de las dos curvas
4. **El cociente es el factor de corrección del F₀ objetivo**

### 10.16.2 Recomendación sobre indicadores rutinarios

Sin cambiar el proceso, sustituir la cinta Clase 1 por **indicadores Clase 5
(integradores)**, que sí responden a tiempo y temperatura conjuntamente, y usar
**indicadores biológicos** periódicamente como verificación.

---

## 11. OBSERVACIONES ABIERTAS

### 11.1 Altitud del sitio — RESUELTO, con implicación operativa

**Ubicación: Medellín, Parque Explora, ~1495 m sobre el nivel del mar.**

| | Nivel del mar | Medellín (1495 m) |
| :--- | :--- | :--- |
| Presión atmosférica | 101.3 kPa | **84.6 kPa** |
| Ebullición del agua | 100.0 °C | **94.9 °C** |
| Manométrica para 121.1 °C | 15.1 psi | **17.5 psi** |
| Temperatura a 15 psi manométricos | 121.1 °C | **~118.3 °C** |

**Techo de presión: no es un problema.** El Sr. Carlos confirma que el equipo ya
operaba y alcanzaba 121 °C, luego el tarado de la válvula de seguridad del 75X
está por encima de los 17.5 psi necesarios.

**Implicación que sí queda vigente:** la regla de oro *"15 psi = 121 °C"* no
aplica en Medellín. A 15 psi manométricos el vapor está a ~118.3 °C. Para 121.1 °C
reales hace falta ~17.5 psi en el manómetro. Documentarlo en el manual de uso.

**Por qué F₀ lo absorbe automáticamente:** a 118.3 °C la tasa de letalidad es
`10^((118.3-121.1)/10) = 0.52` min/min, la mitad. El ciclo simplemente tarda
~38 min en vez de ~20 en llegar a F₀ = 20. Con un temporizador fijo se habría
esterilizado a la mitad de la letalidad sin que nadie lo notara. Es el caso que
justifica la decisión de §5 Fase 3.

**Calibración:** la referencia de agua en ebullición es **94.94 °C**, no 100 °C.
Ya reflejado en `ALTITUD_M = 1495.0` en el sketch del paso 1.

### 11.2 Margen de disparo del SSR a 3.3 V — MEDIDO

Medido con GPIO25 en alto fijo, SSR conectado (10 kΩ de pull-down en su sitio),
sin AC:

| | Valor |
| :--- | ---: |
| Tensión en la entrada del SSR (bornes 3-4) | **3.092 V** |
| Mínimo de especificación del SSR-25 DA | 3.0 V |
| Margen sobre el mínimo | **~0.09 V (3 %)** |
| LED indicador del SSR | Encendido |

La caída de ~0.2 V respecto al rail de 3.3 V implica que el GPIO entrega del
orden de 8–15 mA, dentro de lo seguro para el ESP32 (20 mA recomendado).

**Veredicto: funciona, pero sin holgura.** El optoacoplador dispara. Queda
pendiente confirmar en la fase B que la conmutación de la carga AC es limpia.

**Si en algún momento aparece parpadeo errático o disparo intermitente** — por
calentamiento del gabinete, por sustitución del SSR por otra unidad, o al añadir
consumo al rail de 3.3 V — la solución es manejar la entrada del SSR desde los
5 V de la fuente con un MOSFET de señal (2N7000 o similar) o un NPN pequeño,
manteniendo el pull-down de 10 kΩ en el lado del ESP32. El BOM ya incluye la
fuente de 5 V, así que el cambio es de un componente.

Revisar esta medida otra vez con el gabinete cerrado y a temperatura de régimen.
