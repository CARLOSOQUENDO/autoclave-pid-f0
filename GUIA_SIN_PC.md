# GUÍA DE OPERACIÓN SIN PC — firmware v0.5.0

> Todo lo de este documento se hace **solo con el encoder y la pantalla**.
> El PC no hace falta para nada de esto.
>
> Pruebas con la **olla abierta, sin presión**.

---

## 0. NAVEGACIÓN

| Gesto | Qué hace |
| :--- | :--- |
| **Clic corto** | Entrar / seleccionar / confirmar |
| **Girar** | Mover el cursor, o cambiar el valor si estás editando |
| **Pulsación larga** (>0.6 s) | Volver atrás. **Y aborta el ensayo si está corriendo** |

Desde la pantalla principal, un clic entra al menú.

---

## 1. ENSAYO DE ESCALÓN — el que hay que hacer primero

Es la medición que da los parámetros de la planta. **Sin esto, el PID no se
puede afinar.**

```
MENU → Ensayo planta >
          Salida          50 %      ← clic, girar, clic
          Duracion        45 min    ← clic, girar, clic
          Iniciar ensayo            ← clic
       CONFIRMAR: "Ensayo 50% / 45 min"      (arranca en NO)
          girar a la derecha = SI, clic
       VERIFICACION 1/4 … 4/4                (un NO cancela todo)
       → arranca
```

**Durante el ensayo la pantalla muestra:** temperatura grande, minutos
transcurridos y totales, incremento desde el inicio, número de muestras, y una
**gráfica de la curva completa**.

**Al terminar:** suena una melodía y pasa sola a **RESULTADOS** con L, τ, K y
los valores de Kp y Ki sugeridos.

### Qué esperar (1.680 W medidos)

| Momento | Subida esperada |
| :--- | :--- |
| Primeros 1-2 min | Casi nada — es el tiempo muerto |
| A los 10 min | **+12 a +15 °C** |
| A los 45 min | Cerca de la ebullición (94.9 °C) |

Si a los 10 minutos ha subido menos de +5 °C, algo va mal: abortar y revisar.

### ⚠️ Anota estos números de la pantalla RESULTADOS

```
L    ______ s
tau  ______ s
K    ______
Kp   ______   Ki  ______
```

**Apúntalos en papel.** Son el resultado del día y hacen falta para el paso 2.

---

## 2. PRUEBA DEL CONTROL PID — solo después del paso 1

Con los Kp y Ki que salieron en RESULTADOS, se prueba si el lazo mantiene una
temperatura estable.

### Primero, cargar los parámetros

```
MENU → Configuracion >
          Control Kp      ← clic, girar hasta el valor de RESULTADOS, clic
          Control Ki      ← igual
          Control Kd      ← dejar en 0 (§6: el derivativo estorba aquí)
          Guardar         ← clic. IMPORTANTE, si no se pierden al reiniciar
```

### Elegir la temperatura objetivo

```
MENU → Temperatura        ← clic, girar, clic
```

> ⚠️ **Con la olla abierta no se puede pasar de 94.9 °C**, que es donde hierve
> el agua en Medellín. Para probar el PID, pon el objetivo en **80 °C**. Un
> setpoint por encima de la ebullición haría que el lazo se quede al 100 %
> para siempre sin poder llegar, y no se aprende nada.

### Lanzarlo

```
MENU → Ensayo planta > → Control PID    ← clic
       CONFIRMAR: "PID a 80.0 C / 45 min"
       VERIFICACION 1/4 … 4/4
       → arranca
```

**La pantalla muestra:** temperatura grande, `SP` (objetivo), `err` (error con
signo), barra de salida con el porcentaje, y la curva.

### Qué mirar

| Comportamiento | Significa | Qué hacer |
| :--- | :--- | :--- |
| Llega y se queda estable, error < 0.5 °C | **Bien afinado** | Nada, anotar los valores |
| Se pasa mucho y luego baja | Kp demasiado alto | Bajar Kp a la mitad |
| Tarda muchísimo en llegar | Kp demasiado bajo | Subir Kp |
| Oscila arriba y abajo sin parar | Kp o Ki demasiado altos | Bajar Ki primero |
| Se queda por debajo y no sube | Ki demasiado bajo | Subir Ki |

Cambia **un parámetro a la vez** y anota qué pasó. Es la única forma de saber
cuál hizo qué.

---

## 3. SEGURIDAD — lo que el equipo hace solo

| Protección | Cuándo actúa |
| :--- | :--- |
| Límite absoluto | T > 128 °C → corte incondicional |
| Sensor no válido | RTD abierta, en corto, o salto imposible → corte |
| Marcha en seco | Salida ≥80 % durante 5 min sin subir 5 °C → corte |
| Estado de FALLA | **Terminal.** Solo se sale reiniciando el equipo |

> El detector de marcha en seco **no vigila en la meseta de ebullición**
> (94.9 ± 3 °C), porque ahí el estancamiento es normal. Vuelve a vigilar en
> cuanto la temperatura sube por encima — que es la firma real del seco.

### Si aparece la pantalla de FALLA

Muestra el motivo. La salida ya está cortada. **Para salir del estado de falla
hay que reiniciar el equipo** (quitar y poner alimentación).

Sin PC no se puede usar el comando `reset`, así que el reinicio es la vía.

---

## 4. MEDIDAS CON INSTRUMENTO — pendientes del checklist

Aprovecha cualquier corrida para tomarlas:

| Qué | Cuándo | Umbral |
| :--- | :--- | :--- |
| **Infrarrojo en la carcasa del SSR** | A mitad de ensayo | **>80 °C → abortar** |
| **Infrarrojo en el termostato y la pared de la olla** | Cuando la PT100 marque ~90 °C | Diferencia <5 °C respecto a la PT100 |

La del SSR es la crítica: con 14 A disipa ~17 W, y un SSR recalentado acaba
**fallando en cortocircuito**.

---

## 5. LÍMITES DE ESTA VERSIÓN

Lo que **todavía no existe** (llega en el paso 6):

- Ciclo de esterilización completo (purga → rampa → meseta)
- Acumulación de letalidad F₀
- Confirmación de cierre de la válvula de purga

El menú tiene "Letalidad F0" como parámetro editable, pero **aún no se usa**.
Se guarda para cuando exista el ciclo.

---

## 6. LOS DATOS QUEDAN GUARDADOS

- La curva se vuelca a la memoria flash **cada 2 minutos** y al terminar
- L, τ y K sobreviven al reinicio: se ven en **Ver resultados**
- Cuando vuelva a haber PC, el comando `volcar` saca el CSV completo

Un corte de energía a mitad de ensayo pierde como mucho los últimos 2 minutos.

---

## RESUMEN DE UNA LÍNEA

**Hoy: ensayo de escalón 50 % / 45 min → anota L, τ, K, Kp, Ki de la pantalla
RESULTADOS. Luego: mételos en Configuración, Guardar, y prueba el PID a 80 °C.**
