#!/usr/bin/env python3
"""
registrar.py - Registro y analisis del ensayo de escalon (paso 4)

Abre el puerto UNA sola vez y lo mantiene abierto durante todo el ensayo. Es
critico: el circuito de auto-reset del DevKit se dispara al activar DTR/RTS, asi
que reconectarse a mitad de la curva resetea el ESP32 y pierde el ensayo.

Uso:
    python registrar.py --duty 50 --minutos 45
    python registrar.py --analizar ensayos/escalon_20260905_1430.csv

Al terminar guarda el CSV en ensayos/ y extrae los parametros FOPDT
(L = tiempo muerto, tau = constante de tiempo, K = ganancia) por el metodo de la
tangente, que es lo que necesita sTune / el PID del paso 5.
"""

import argparse
import csv
import os
import re
import sys
import time
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("Falta pyserial:  pip install pyserial")

BAUDIOS = 115200
DIR_ENSAYOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ensayos")

# Adaptadores USB-serie habituales en los DevKit ESP32
PATRON_PUERTO = re.compile(r"CH340|CP210|CH910|Silicon Labs|USB Serial|UART", re.I)


def detectar_puerto():
    """Devuelve el puerto del ESP32, o None si no hay uno claro."""
    candidatos = [p for p in list_ports.comports()
                  if PATRON_PUERTO.search(f"{p.description} {p.manufacturer or ''}")]
    if not candidatos:
        return None
    if len(candidatos) > 1:
        print("Varios puertos candidatos:")
        for p in candidatos:
            print(f"   {p.device}  {p.description}")
        return None
    return candidatos[0].device


class Ensayo:
    """Mantiene el puerto abierto de principio a fin."""

    def __init__(self, puerto):
        self.ser = serial.Serial(puerto, BAUDIOS, timeout=0.5)
        # Pulso de reset controlado: preferimos arrancar de cero a propósito
        # antes que sufrir un reset inesperado a mitad del ensayo.
        self.ser.dtr = False
        self.ser.rts = True
        time.sleep(0.1)
        self.ser.rts = False
        time.sleep(0.8)
        self.ser.reset_input_buffer()
        self.filas = []

    def enviar(self, cmd):
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()

    def leer_linea(self):
        linea = self.ser.readline()
        if not linea:
            return None
        return linea.decode("utf-8", errors="replace").rstrip()

    def cerrar(self):
        try:
            self.enviar("parar")
            time.sleep(0.3)
            self.enviar("desarmar")
            time.sleep(0.3)
        finally:
            self.ser.close()


def es_fila_csv(linea):
    """El firmware emite '#' para mensajes y CSV puro para datos."""
    if not linea or linea.startswith("#"):
        return None
    partes = linea.split(",")
    if len(partes) != 5:
        return None
    try:
        return (int(partes[0]), float(partes[1]), int(partes[2]),
                float(partes[3]), int(partes[4]))
    except ValueError:
        return None


def correr(puerto, duty, minutos):
    os.makedirs(DIR_ENSAYOS, exist_ok=True)
    marca = datetime.now().strftime("%Y%m%d_%H%M")
    ruta = os.path.join(DIR_ENSAYOS, f"escalon_{duty}pct_{marca}.csv")

    ens = Ensayo(puerto)
    print(f"Puerto {puerto} abierto. NO lo cierres ni abras otro monitor serie.")
    print(f"Registrando en {ruta}\n")

    # Deja pasar el arranque
    t0 = time.time()
    while time.time() - t0 < 4:
        l = ens.leer_linea()
        if l:
            print(l)

    ens.enviar("armar")
    time.sleep(1.0)
    ens.enviar(f"escalon {duty} {minutos}")

    limite = time.time() + minutos * 60 + 60
    en_falla = False

    try:
        while time.time() < limite:
            linea = ens.leer_linea()
            if linea is None:
                continue

            fila = es_fila_csv(linea)
            if fila:
                t_s, temp, d, d_real, estado = fila
                ens.filas.append(fila)
                if estado == 1:
                    print(f"  t={t_s:5d}s  T={temp:7.3f} C  duty={d:3d}%  "
                          f"real={d_real:6.2f}%")
                if estado == 2 and not en_falla:
                    en_falla = True
                    print("\n*** El firmware entro en FALLA. Ensayo abortado. ***")
                    break
                if estado == 0 and t_s == 0 and len(ens.filas) > 10:
                    print("\nEnsayo terminado por el firmware.")
                    break
            else:
                print(linea)
    except KeyboardInterrupt:
        print("\nInterrumpido por el operador.")
    finally:
        ens.cerrar()

    if not ens.filas:
        print("No se registro ningun dato.")
        return

    with open(ruta, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "T_C", "duty_pct", "duty_real_pct", "estado"])
        w.writerows(ens.filas)
    print(f"\nGuardadas {len(ens.filas)} muestras en {ruta}")

    analizar(ruta)


def analizar(ruta):
    """Extrae L, tau y K de la curva de escalon por el metodo de la tangente."""
    with open(ruta, newline="", encoding="utf-8") as f:
        filas = [r for r in csv.DictReader(f)]

    datos = [(float(r["t_s"]), float(r["T_C"]), int(r["duty_pct"]))
             for r in filas if int(r["estado"]) == 1]
    if len(datos) < 30:
        print("Muestras insuficientes para analizar (hacen falta al menos 30).")
        return

    t = [d[0] for d in datos]
    T = [d[1] for d in datos]
    duty = datos[0][2]

    T0, Tf = T[0], T[-1]
    dT = Tf - T0

    print("\n" + "=" * 58)
    print(" ANALISIS DE LA RESPUESTA AL ESCALON")
    print("=" * 58)
    print(f"Duty aplicado    : {duty} %")
    print(f"T inicial / final: {T0:.2f} / {Tf:.2f} C   (dT = {dT:+.2f} C)")
    print(f"Duracion         : {t[-1]:.0f} s ({t[-1]/60:.1f} min)")

    if dT < 5:
        print("\nLa temperatura apenas subio: la curva no sirve para identificar")
        print("la planta. Repetir con mas duty o mas tiempo.")
        return

    # Pendiente maxima sobre una ventana movil de 30 s, robusta al ruido
    ventana = max(3, int(30 / max(1, t[1] - t[0])))
    mejor_pend, mejor_i = 0.0, 0
    for i in range(len(t) - ventana):
        p = (T[i + ventana] - T[i]) / (t[i + ventana] - t[i])
        if p > mejor_pend:
            mejor_pend, mejor_i = p, i

    if mejor_pend <= 0:
        print("\nNo se detecto pendiente positiva. Revisar el ensayo.")
        return

    # Tangente en el punto de inflexion: corta T0 en L, y Tf en L + tau
    t_infl = t[mejor_i + ventana // 2]
    T_infl = T[mejor_i + ventana // 2]
    L   = t_infl - (T_infl - T0) / mejor_pend
    tau = (Tf - T0) / mejor_pend
    K   = dT / duty          # C por % de salida

    L = max(L, 0.0)

    print(f"\nPendiente maxima : {mejor_pend*60:.3f} C/min")
    print(f"L   (tiempo muerto)   : {L:8.1f} s  ({L/60:.2f} min)")
    print(f"tau (cte. de tiempo)  : {tau:8.1f} s  ({tau/60:.2f} min)")
    print(f"K   (ganancia)        : {K:8.4f} C/%")

    if L > 0:
        print(f"Controlabilidad tau/L : {tau/L:8.2f}", end="")
        r = tau / L
        if r > 10:  print("   (facil de controlar)")
        elif r > 4: print("   (normal)")
        else:       print("   (dificil: mucho tiempo muerto)")

    # Ziegler-Nichols para PI en lazo abierto, como punto de partida de sTune
    if L > 0 and K > 0:
        kp = 0.9 * tau / (K * L)
        ti = 3.33 * L
        ki = kp / ti
        print("\nPunto de partida PI (Ziegler-Nichols, lazo abierto):")
        print(f"   Kp = {kp:.3f}")
        print(f"   Ki = {ki:.5f}   (Ti = {ti:.0f} s)")
        print("   Kd = 0        (sec.6: el derivativo estorba con el cambio de fase)")
        print("\nSon valores de arranque, no finales. Refinar con sTune en la")
        print("zona de 110-121 C (sec.6), nunca desde temperatura ambiente.")

    # Aviso sobre la meseta de ebullicion
    if T0 < 90 and Tf > 93:
        print("\nAVISO: la curva atraviesa el punto de ebullicion (94.9 C en")
        print("Medellin). La dinamica antes y despues del cambio de fase es")
        print("completamente distinta, asi que estos parametros mezclan dos")
        print("plantas. Para el PID de la meseta, repetir el escalon partiendo")
        print("de 105-110 C y con la olla ya cerrada.")
    print("=" * 58)


def main():
    ap = argparse.ArgumentParser(description="Ensayo de escalon del autoclave")
    ap.add_argument("--puerto", help="COMx (por defecto, autodetectar)")
    ap.add_argument("--duty", type=int, default=50, help="duty en %% (1-100)")
    ap.add_argument("--minutos", type=int, default=45, help="duracion")
    ap.add_argument("--analizar", metavar="CSV",
                    help="analiza un CSV ya registrado y sale")
    args = ap.parse_args()

    if args.analizar:
        analizar(args.analizar)
        return

    puerto = args.puerto or detectar_puerto()
    if not puerto:
        sys.exit("No se detecto el ESP32. Indica el puerto con --puerto COMx")

    print("=" * 58)
    print(" ENSAYO DE ESCALON - PID AUTOCLAVE 75X")
    print("=" * 58)
    print(f"Puerto : {puerto}")
    print(f"Duty   : {args.duty} %")
    print(f"Tiempo : {args.minutos} min")
    print("\nANTES DE CONTINUAR, verifica:")
    print("  - Termostato bimetalico de 130 C en serie con la fase")
    print("  - Fusible en la fase")
    print("  - SSR sobre disipador con pasta termica")
    print("  - Valvula de seguridad libre")
    print("  - Nivel de agua correcto, resistencia sumergida")
    print("  - Vas a permanecer presente durante todo el ensayo")
    if input("\nEscribe SI para arrancar: ").strip().upper() != "SI":
        sys.exit("Cancelado.")

    correr(puerto, args.duty, args.minutos)


if __name__ == "__main__":
    main()
