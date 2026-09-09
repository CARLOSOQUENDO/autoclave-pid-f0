# CHECKLIST — PRIMER ENSAYO CON RESISTENCIA REAL

> **Autoclave All American 75X — paso 4, caracterización de planta**
>
> Este es el primer momento en que se energiza la resistencia dentro de un
> recipiente a presión. Recorrer el checklist entero, en orden, marcando cada
> punto. Cualquier casilla que no se pueda marcar **detiene el ensayo**.
>
> Fecha del ensayo: ____________  Operador: ____________________

---

## A. CON EL EQUIPO DESENCHUFADO DE LA RED

Nada de esto se mide con tensión presente. Desenchufar y verificar que está
desenchufado antes de tocar bornes.

### A.1 Resistencia del elemento calefactor — resuelve §1.1

Medir entre los dos bornes del elemento, con sus cables desconectados de todo lo
demás.

| Lectura | Escenario | Corriente | Fusible | Disipador |
| :--- | :--- | :--- | :--- | :--- |
| ~23.7 Ω | A — 607 W | 5.1 A | 10 A | La carcasa basta |
| ~8.7 Ω | B — 1650 W | 13.8 A | 20 A | **Con aletas, obligatorio** |

- [ ] Medida obtenida: __________ Ω → escenario ______

> Si sale un valor intermedio, anotarlo y **no continuar** hasta aclararlo: el
> dimensionado del fusible y de la disipación dependen de esto.

### A.2 Aislamiento del elemento contra la vaina — CRÍTICO

Es un calentador de inmersión: si su aislamiento está degradado, deriva corriente
al agua y a la olla. Medir entre **cada borne del elemento** y **el cuerpo
metálico de la olla**.

- [ ] Borne 1 → cuerpo de la olla: **circuito abierto** (OL / >2 MΩ)
- [ ] Borne 2 → cuerpo de la olla: **circuito abierto** (OL / >2 MΩ)

> Cualquier lectura por debajo de 1 MΩ es fuga a masa. **No energizar.**

### A.3 Termostato bimetálico

- [ ] Continuidad entre sus bornes en frío: **cerrado**, ~0 Ω (es NC)
- [ ] Está en serie con la **fase**, no con el neutro
- [ ] Su cable es siliconado o con camisa de fibra de vidrio

### A.4 Tierra y aislamiento general

- [ ] Chasis del gabinete → pin de tierra del enchufe: **< 1 Ω**
- [ ] Cuerpo de la olla → tierra: **< 1 Ω**
- [ ] Fase → chasis: circuito abierto
- [ ] Neutro → chasis: circuito abierto

### A.5 Lado de control

- [ ] Pull-down entre GPIO25 y GND: **10 kΩ** (§3.1-B)
- [ ] Fusible del valor correcto según A.1, y con continuidad
- [ ] Fusible instalado en la **fase**, antes de todo lo demás
- [ ] Todos los tornillos de bornera apretados; ningún hilo suelto

---

## B. VERIFICACIÓN MECÁNICA

### B.1 Termostato bimetálico (§10.6)

- [ ] Cara plana contra la pared de la olla, con **pasta térmica** >150 °C
- [ ] Sujeto con abrazadera inox o fleje — **no con cinta ni adhesivo**
- [ ] Situado **por encima del nivel del agua**, en la zona de vapor
- [ ] **Cubierto con lana cerámica o fibra de vidrio** ← lo que más se olvida

> Sin aislante encima, el termostato pierde calor al aire ambiente y se queda
> 10–20 °C por debajo de la pared que toca: cortaría demasiado tarde.

### B.2 SSR

- [ ] Atornillado al disipador con pasta térmica, capa fina y uniforme
- [ ] Disipador con aletas si aplica el escenario B (§1.1)
- [ ] Los bornes 1 y 2 (potencia) no tocan nada del chasis

### B.3 Olla

- [ ] **Válvula de seguridad libre**, sin obstrucción — verificar moviéndola
- [ ] Nivel de agua correcto, **resistencia completamente sumergida**
- [ ] Tapa asegurada y junta en buen estado
- [ ] Válvula de purga accesible y operable

### B.4 Cableado

- [ ] Cable de la PT100 **separado** del cableado de potencia
- [ ] Prensaestopas apretados en ambas entradas
- [ ] Nada suelto ni apoyado sobre superficies calientes dentro del gabinete
- [ ] Gabinete cerrado antes de energizar

---

## C. VERIFICACIÓN DEL FIRMWARE

### C.1 Ya comprobado en banco

| Comprobación | Resultado |
| :--- | :--- |
| Arranque en `IDLE`, salida desarmada, SSR en LOW | ✅ |
| Sensor PT100 leyendo, estado `[OK]` | ✅ |
| Límite absoluto de 128 °C configurado | ✅ |
| `escalon` rechazado con la salida desarmada | ✅ |
| `reset` sin falla activa lo reporta correctamente | ✅ |

### C.2 Prueba en vivo de la cadena de protección — HACER SIEMPRE

Esto valida de extremo a extremo que la protección funciona, **antes** de
confiarle la resistencia. Con la olla **desenchufada de la red** y solo el USB
conectado:

1. [ ] Ejecutar `armar` → responde `*** SALIDA ARMADA ***`
2. [ ] **Desconectar físicamente un hilo de la PT100** del MAX31865
3. [ ] En menos de 2 segundos debe ocurrir todo esto:
      - [ ] Mensaje `*** FALLA: flag de falla del MAX31865 ***` (o `lectura NaN`)
      - [ ] **5 pitidos graves** del buzzer
      - [ ] La pantalla muestra `FALLA` y el motivo
      - [ ] `estado` reporta `Salida: desarmada`
4. [ ] Reconectar la PT100
5. [ ] Verificar que **NO se recupera sola**: sigue en FALLA (es terminal, §7.2)
6. [ ] Ejecutar `reset` → vuelve a `IDLE`, **pero sigue desarmada**

> Si algún punto de C.2 falla, **no continuar**. Es la única red de seguridad del
> firmware y tiene que demostrarse antes, no durante.

---

## D. DURANTE EL ENSAYO

### D.1 Antes de arrancar

- [ ] Sé cómo cortar la energía en 2 segundos (desenchufar la regleta)
- [ ] La regleta está accesible, no detrás del equipo
- [ ] Voy a permanecer presente **todo el ensayo** (45–60 min)
- [ ] Tengo a mano: pinza amperimétrica y termómetro infrarrojo

### D.2 Arranque — desde el menú, sin PC

El equipo es autónomo: registra, analiza y muestra resultados por su cuenta.

```
Pantalla principal
   │ clic
   ▼
MENU  →  Ensayo planta  >
            Salida            50 %     ← ajustar con el encoder
            Duracion          45 min
            Iniciar ensayo             ← clic
   │
   ▼  CONFIRMAR: "Energiza la resistencia"   (arranca en NO)
   │  girar derecha = SI, clic
   ▼  VERIFICACION 1/4 ... 4/4            (un NO cancela todo)
   │
   ▼  El ensayo arranca. Pantalla en vivo con la curva.
```

**Durante el ensayo la pantalla muestra:** temperatura grande, minutos
transcurridos/totales, incremento desde el inicio, número de muestras y una
**gráfica de la curva completa** reescalada.

**Para abortar:** pulsación larga del encoder. O desenchufar, si hay urgencia.

Al terminar suena una melodía y pasa sola a la pantalla de **RESULTADOS** con
L, τ, K y el PI propuesto.

> El PC solo hace falta después, para sacar el CSV con el comando `volcar`.

### D.3 Medidas a tomar durante el ensayo

**Corriente — resuelve §1.1 definitivamente**

- [ ] Pinza amperimétrica en **un solo conductor** de la resistencia, mientras el
      SSR está conduciendo. Corriente medida: __________ A

**Temperatura del termostato — verifica §10.6.1**

Con la PT100 marcando ~100 °C y otra vez a ~120 °C, medir con infrarrojo:

| PT100 (interior) | Pared junto al termostato | Cuerpo del termostato |
| :--- | :--- | :--- |
| ~100 °C | __________ | __________ |
| ~120 °C | __________ | __________ |

| Diferencia respecto a la PT100 | Conclusión |
| :--- | :--- |
| < 5 °C | El termostato de 130 °C es correcto |
| 5–10 °C | Mejorar aislamiento y repetir |
| > 10 °C | Cortaría tarde. Cambiar a uno de 110–115 °C |

**Temperatura del SSR**

- [ ] Carcasa del SSR con infrarrojo a mitad del ensayo: __________ °C

> Por encima de 80 °C la disipación es insuficiente. Abortar y mejorar el
> disipador antes de seguir.

### D.4 Vigilancia continua

- [ ] Olor a quemado o a aislamiento caliente
- [ ] Ruido anormal en la olla
- [ ] Vapor saliendo por donde no debe
- [ ] La temperatura sube de forma continua y coherente
- [ ] El manómetro de la olla acompaña a la temperatura

---

## E. CRITERIOS DE ABORTO INMEDIATO

**Desenchufar la regleta sin dudar** si ocurre cualquiera de estos:

| Señal | Por qué |
| :--- | :--- |
| Olor a quemado | Aislamiento fallando |
| Humo, chispas o zumbido en el gabinete | Falla eléctrica en curso |
| Carcasa del SSR por encima de 80 °C | Disipación insuficiente, va a fallar en corto |
| La temperatura sigue subiendo con duty 0 % | **SSR pegado en corto** |
| El firmware entra en FALLA y la temperatura sigue subiendo | Falló la protección primaria |
| Vapor por la junta de la tapa | Sellado defectuoso bajo presión |
| Cualquier cosa que no entiendas | La duda es motivo suficiente |

> **El caso más grave es la temperatura subiendo con duty 0 %**: significa que el
> SSR conduce sin orden. El termostato bimetálico debería cortar a 130 °C, pero
> no hay que esperar a comprobarlo — desenchufar de inmediato.

---

## F. AL TERMINAR

- [ ] **No abrir la olla hasta presión cero.** Despresurización natural.
- [ ] Anotar los resultados de D.3 en `CONTEXTO.md`
- [ ] El equipo guarda el CSV en su flash y extrae L, τ y K solo
- [ ] Para sacar los datos al PC: conectar USB y ejecutar el comando `volcar`
- [ ] Revisar el estado del SSR y del termostato en frío

**Resultados del análisis:**

- L (tiempo muerto): __________ s
- τ (constante de tiempo): __________ s
- K (ganancia): __________ °C/%
