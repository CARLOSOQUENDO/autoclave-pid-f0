#!/usr/bin/env python3
"""
simular_pantallas.py - Previsualiza las pantallas de la OLED sin tocar el equipo

Dibuja los mismos 128x64 pixeles que la SSD1306, respetando las metricas reales
de las fuentes U8g2 que usa el firmware. Sirve para revisar textos y detectar
solapes ANTES de compilar y subir, que es como se colo el solape de la purga.

Uso:
    python herramientas/simular_pantallas.py
    python herramientas/simular_pantallas.py purga

Genera PNG en herramientas/pantallas/ ampliados x5.
"""

import os
import sys
from PIL import Image, ImageDraw

ANCHO, ALTO, ESCALA = 128, 64, 5
SALIDA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pantallas")

# Metricas de las fuentes U8g2 usadas en el firmware: (ancho, alto, ascenso)
FUENTES = {
    "5x8":    (5,  8,  7),
    "6x10":   (6, 10,  8),
    "7x13":   (7, 13, 10),
    "logi20": (14, 20, 20),   # logisoso20_tn, solo digitos y ':' '.' '-'
}


class Lienzo:
    """Emula las primitivas de U8g2 sobre un buffer de 128x64."""

    def __init__(self):
        self.img = Image.new("1", (ANCHO, ALTO), 0)
        self.d = ImageDraw.Draw(self.img)
        self.fuente = "6x10"
        self.color = 1
        self.avisos = []

    def setFont(self, f):
        self.fuente = f

    def setDrawColor(self, c):
        self.color = c

    def anchoTexto(self, t):
        return len(t) * FUENTES[self.fuente][0]

    def drawStr(self, x, y, t):
        """y es la LINEA BASE, como en U8g2."""
        an, al, asc = FUENTES[self.fuente]
        arriba = y - asc
        if arriba < 0:
            self.avisos.append(f"'{t[:18]}' se sale por arriba (y={y})")
        if y > ALTO:
            self.avisos.append(f"'{t[:18]}' se sale por abajo (y={y})")
        ancho = self.anchoTexto(t)
        if x + ancho > ANCHO:
            sobra = x + ancho - ANCHO
            self.avisos.append(f"'{t[:18]}' se sale {sobra} px por la derecha")
        # Cada caracter como un bloque, para ver ocupacion real
        for i, c in enumerate(t):
            if c == " ":
                continue
            cx = x + i * an
            self.d.rectangle([cx, arriba + 1, cx + an - 2, y - 1], fill=self.color)
        return (x, arriba, x + ancho, y)

    def drawHLine(self, x, y, w):
        self.d.rectangle([x, y, x + w - 1, y], fill=self.color)

    def drawFrame(self, x, y, w, h):
        self.d.rectangle([x, y, x + w - 1, y + h - 1], outline=self.color)

    def drawBox(self, x, y, w, h):
        self.d.rectangle([x, y, x + w - 1, y + h - 1], fill=self.color)

    def guardar(self, nombre, texto_plano):
        os.makedirs(SALIDA, exist_ok=True)
        g = self.img.convert("L").point(lambda v: 255 if v else 0)
        fondo = Image.new("RGB", (ANCHO, ALTO), (10, 20, 40))
        azul = Image.new("RGB", (ANCHO, ALTO), (120, 210, 255))
        fondo.paste(azul, (0, 0), g)
        fondo = fondo.resize((ANCHO * ESCALA, ALTO * ESCALA), Image.NEAREST)
        ruta = os.path.join(SALIDA, f"{nombre}.png")
        fondo.save(ruta)
        return ruta


def marco(titulo, lineas, avisos):
    """Version en texto, que es la que se puede leer aqui mismo."""
    print(f"\n  {titulo}")
    print("  +" + "-" * 30 + "+")
    for l in lineas:
        print(f"  |{l[:30]:<30}|")
    print("  +" + "-" * 30 + "+")
    for a in avisos:
        print(f"  [!] {a}")


# ---------------------------------------------------------------------------
# PANTALLAS
# ---------------------------------------------------------------------------

def purga(c, T=96.8, total=765, resta=272):
    c.setFont("6x10")
    c.drawStr(0, 8, "PURGA")
    t = f"{total//60}:{total%60:02d}"
    c.drawStr(128 - c.anchoTexto(t), 8, t)
    c.drawHLine(0, 11, 128)

    c.setFont("logi20")
    c.drawStr(2, 33, f"{resta//60}:{resta%60:02d}")
    c.setFont("6x10")
    tt = f"{T:.1f}C"
    c.drawStr(128 - c.anchoTexto(tt), 31, tt)

    c.drawFrame(0, 37, 128, 8)
    c.drawBox(1, 38, 70, 6)

    c.setFont("5x8")
    c.drawStr(0, 54, "DEBE SALIR VAPOR CONTINUO")
    c.drawStr(0, 63, "POR LA VALVULA ABIERTA")
    return ["PURGA                   12:45",
            "------------------------------",
            "  4:32              96.8C",
            " [######              ]",
            "------------------------------",
            "DEBE SALIR VAPOR CONTINUO",
            "POR LA VALVULA ABIERTA"]


def meseta(c, T=121.2, F0=12.4, obj=20, duty=38, total=2312):
    c.setFont("6x10")
    c.drawStr(0, 8, "ESTERILIZANDO")
    t = f"{total//60}:{total%60:02d}"
    c.drawStr(128 - c.anchoTexto(t), 8, t)
    c.drawHLine(0, 11, 128)

    c.setFont("logi20")
    c.drawStr(2, 34, f"{T:.1f}")
    c.setFont("5x8")
    s = "META 121.0"
    c.drawStr(128 - c.anchoTexto(s), 22, s)
    d = f"{duty}%"
    c.drawStr(128 - c.anchoTexto(d), 32, d)

    c.setFont("6x10")
    c.drawStr(0, 46, f"F0 {F0:.1f} / {obj:.0f}")
    c.drawFrame(0, 50, 128, 9)
    c.drawBox(1, 51, 78, 7)
    c.setFont("5x8")
    c.drawStr(0, 63, "SUMA 1.05 F0 POR MINUTO")
    return ["ESTERILIZANDO           38:32",
            "------------------------------",
            " 121.2          META 121.0",
            "                        38%",
            "F0 12.4 / 20",
            " [############        ]",
            "SUMA 1.05 F0 POR MINUTO"]


def cerrar(c, T=96.9, total=800, avisos=0):
    c.setFont("6x10")
    c.drawStr(0, 8, "CERRAR VALVULA")
    t = f"{total//60}:{total%60:02d}"
    c.drawStr(128 - c.anchoTexto(t), 8, t)
    c.drawHLine(0, 11, 128)

    c.drawBox(0, 13, 128, 17)
    c.setDrawColor(0)
    c.setFont("7x13")
    c.drawStr(6, 26, "CIERRE LA VALVULA")
    c.setDrawColor(1)

    c.setFont("5x8")
    c.drawStr(0, 41, f"{T:.1f} C   PURGA LISTA")
    c.drawStr(0, 54, "CIERRE LA VALVULA AHORA")
    return ["CERRAR VALVULA          13:20",
            "------------------------------",
            "###  CIERRE LA VALVULA    ###",
            "------------------------------",
            "96.9 C   PURGA LISTA",
            "",
            "CIERRE LA VALVULA AHORA"]



CHECKLIST = [
    ("VALVULA DE PURGA",     "ABIERTA"),
    ("NIVEL DE AGUA ALTO,",  "RESISTENCIA CUBIERTA"),
    ("VALVULA DE SEGURIDAD", "LIBRE Y SIN OBSTRUIR"),
    ("TAPA BIEN CERRADA",    "Y ASEGURADA"),
    ("SUSTRATO HUMEDO",      "SECO NO SE ESTERILIZA"),
    ("VOY A ESTAR PRESENTE", "TODO EL CICLO"),
]


def _sino(c, y, sel=0):
    """Selector NO / SI. Arranca SIEMPRE en NO: en un recipiente a presion el
    valor por defecto de cualquier pregunta tiene que ser el que no hace nada."""
    c.setFont("7x13")
    for x, txt, i in ((30, "NO", 0), (86, "SI", 1)):
        if sel == i:
            c.drawBox(x - 7, y - 11, 29, 14)
            c.setDrawColor(0)
        c.drawStr(x, y, txt)
        c.setDrawColor(1)


def checklist(c, n=1):
    l1, l2 = CHECKLIST[n - 1]
    c.setFont("6x10")
    c.drawStr(0, 8, "VERIFICACION")
    t = f"{n}/{len(CHECKLIST)}"
    c.drawStr(128 - c.anchoTexto(t), 8, t)
    c.drawHLine(0, 11, 128)
    c.drawStr(0, 25, l1)
    c.drawStr(0, 36, l2)
    _sino(c, 51)
    c.setFont("5x8")
    t = "NO CANCELA EL CICLO"
    c.drawStr(64 - c.anchoTexto(t) // 2, 63, t)
    return [f"VERIFICACION               {n}/6",
            "------------------------------",
            l1, l2, "",
            "     [NO]        SI",
            "      NO CANCELA EL CICLO"]


def confirmar(c, sp=121):
    c.setFont("6x10")
    c.drawStr(0, 8, "CONFIRMAR")
    c.drawHLine(0, 11, 128)
    c.drawStr(0, 24, "INICIAR CICLO A")
    c.drawStr(0, 35, f"{sp} C  /  F0 20 MIN")
    _sino(c, 51)
    c.setFont("5x8")
    t = "GIRAR ELIGE | CLIC ACEPTA"
    c.drawStr(64 - c.anchoTexto(t) // 2, 63, t)
    return ["CONFIRMAR",
            "------------------------------",
            "INICIAR CICLO A",
            f"{sp} C  /  F0 20 MIN", "",
            "     [NO]        SI",
            "   GIRAR ELIGE | CLIC ACEPTA"]



# ---------------------------------------------------------------------------
# PANTALLAS DE NAVEGACION
# ---------------------------------------------------------------------------

# Solo lo esencial: todo lo que puede alterar el proceso vive en la consola,
# que exige un PC y por tanto la presencia del tecnico.
MENU_OPERADOR = [
    ("INICIAR CICLO",  "",          "ARRANCA LA ESTERILIZACION"),
    ("INFORMACION",    "",          "VERSION Y PARAMETROS"),
    ("VOLVER",         "",          "SALIR AL PANEL PRINCIPAL"),
]

MENU_SERVICIO = [
    ("CALIBRACION",    "",          "AJUSTAR CONTRA EL PATRON"),
    ("TIEMPO MAXIMO",  "150min",    "CORTA EL CICLO POR TIEMPO"),
    ("CONTROL KP",     "4.00",      "GANANCIA PROPORCIONAL"),
    ("CONTROL KI",     "0.0050",    "GANANCIA INTEGRAL"),
    ("CONTROL KD",     "0.00",      "GANANCIA DERIVATIVA"),
    ("VENTANA PWM",    "2000ms",    "PERIODO DE CICLO DEL SSR"),
    ("ENSAYO ESCALON", "",          "CARACTERIZACION DE PLANTA"),
    ("SALIDA ENSAYO",  "50%",       "POTENCIA DEL ESCALON"),
    ("VER RESULTADOS", "",          "L, TAU Y K DEL ULTIMO"),
    ("MODO MANUAL",    "",          "SALIDA MANUAL DE PRUEBAS"),
    ("PROBAR SONIDO",  "",          "COMPRUEBA EL ZUMBADOR"),
    ("GUARDAR",        "",          "GRABA EN MEMORIA"),
    ("VOLVER",         "",          "VUELVE AL MENU ANTERIOR"),
]


def _menu(c, titulo, items, cur):
    """Tres items en fuente grande + linea de ayuda del seleccionado."""
    VIS = 3
    scr = max(0, min(cur - VIS + 1, len(items) - VIS)) if cur >= VIS else 0
    c.setFont("6x10")
    c.drawStr(0, 8, titulo)
    t = f"{cur + 1}/{len(items)}"
    c.drawStr(128 - c.anchoTexto(t), 8, t)
    c.drawHLine(0, 11, 128)

    c.setFont("7x13")
    filas = []
    for i in range(VIS):
        idx = scr + i
        if idx >= len(items):
            break
        y = 24 + i * 13
        etiqueta, valor, _ = items[idx]
        if idx == cur:
            c.drawBox(0, y - 11, 128, 13)
            c.setDrawColor(0)
        c.drawStr(3, y, etiqueta)
        if valor:
            c.drawStr(125 - c.anchoTexto(valor), y, valor)
        c.setDrawColor(1)
        marca = ">" if idx == cur else " "
        filas.append(f"{marca}{etiqueta:<15}{valor:>12}")

    c.drawHLine(0, 53, 128)
    c.setFont("5x8")
    c.drawStr(1, 62, items[cur][2])
    return [f"{titulo:<22}{cur+1:>2}/{len(items)}",
            "------------------------------"] + filas + \
           ["------------------------------", items[cur][2]]


def menu(c, cur=0):
    return _menu(c, "MENU", MENU_OPERADOR, cur)


def servicio(c, cur=0):
    return _menu(c, "SERVICIO", MENU_SERVICIO, cur)


def principal(c, T=26.4, sp=121.0, duty=0, activa=False):
    c.setFont("6x10")
    c.drawStr(0, 8, "ACTIVA" if activa else "EN REPOSO")
    e = "OK"
    c.drawStr(128 - c.anchoTexto(e), 8, e)
    c.drawHLine(0, 11, 128)

    c.setFont("logi20")
    s = f"{T:.1f}"
    an = c.anchoTexto(s)
    c.drawStr(64 - (an + 14) // 2, 38, s)
    c.setFont("6x10")
    c.drawStr(64 + (an + 14) // 2 - 12, 30, "C")

    c.drawStr(0, 50, f"META {sp:.1f}")
    d = f"{duty:3d}%"
    c.drawStr(128 - c.anchoTexto(d), 50, d)
    c.drawFrame(0, 54, 128, 9)
    if duty:
        c.drawBox(1, 55, duty * 126 // 100, 7)
    return ["EN REPOSO                   OK",
            "------------------------------",
            "",
            f"        {T:.1f} C",
            "",
            f"META {sp:.1f}                 {duty}%",
            " [                          ]"]


def info(c, ver="1.5.0"):
    c.setFont("6x10")
    c.drawStr(0, 8, "INFORMACION")
    c.drawHLine(0, 11, 128)

    c.setFont("7x13")
    c.drawStr(2, 23, "PID AUTOCLAVE 75X")
    c.setFont("5x8")
    c.drawStr(2, 33, f"FIRMWARE {ver}")
    c.drawStr(2, 42, "CICLO 121.0 C / F0 20 MIN")
    c.drawStr(2, 50, "1495 m - HIERVE A 94.9 C")

    c.drawHLine(0, 53, 128)
    c.setFont("7x13")
    a = "BY_Oquendo"
    c.drawStr(128 - c.anchoTexto(a) - 3, 64, a)
    return ["INFORMACION",
            "------------------------------",
            "PID AUTOCLAVE 75X",
            f"FIRMWARE {ver}",
            "CICLO 121.0 C / F0 20 MIN",
            "1495 m - HIERVE A 94.9 C",
            "------------------------------",
            "                   BY_Oquendo"]


def editar(c, etiqueta="TEMPERATURA", valor="121.0", unidad="C",
           lo="50.0", hi="135.0"):
    c.setFont("6x10")
    c.drawStr(0, 8, etiqueta)
    c.drawHLine(0, 11, 128)

    c.setFont("logi20")
    w = c.anchoTexto(valor)
    c.setFont("7x13")
    wu = c.anchoTexto(unidad) + 4
    x = 64 - (w + wu) // 2
    c.setFont("logi20")
    c.drawStr(x, 40, valor)
    c.setFont("7x13")
    c.drawStr(x + w + 4, 40, unidad)

    c.setFont("5x8")
    r = f"MIN {lo}   MAX {hi}"
    c.drawStr(64 - c.anchoTexto(r) // 2, 52, r)
    p = "GIRAR CAMBIA | CLIC OK"
    c.drawStr(64 - c.anchoTexto(p) // 2, 62, p)
    return [etiqueta,
            "------------------------------",
            f"        {valor} {unidad}",
            "",
            f"     MIN {lo}   MAX {hi}",
            "    GIRAR CAMBIA | CLIC OK"]


def calibracion(c, cruda=118.42, off=-0.60):
    c.setFont("6x10")
    c.drawStr(0, 8, "CALIBRACION")
    c.drawHLine(0, 11, 128)

    c.setFont("5x8")
    c.drawStr(0, 21, "SIN CORREGIR")
    t = f"{cruda:.2f} C"
    c.drawStr(128 - c.anchoTexto(t), 21, t)
    c.drawStr(0, 31, "OFFSET")
    t = f"{off:+.2f} C"
    c.drawStr(128 - c.anchoTexto(t), 31, t)
    c.drawHLine(0, 35, 128)

    c.drawStr(0, 45, "IGUALAR AL PATRON:")
    c.setFont("logi20")
    v = f"{cruda + off:.2f}"
    c.drawStr(128 - c.anchoTexto(v) - 16, 62, v)
    c.setFont("6x10")
    c.drawStr(114, 54, "C")
    c.setFont("5x8")
    c.drawStr(0, 54, "GIRAR")
    c.drawStr(0, 62, "CLIC=OK")
    return ["CALIBRACION",
            "------------------------------",
            f"SIN CORREGIR          {cruda:.2f} C",
            f"OFFSET                {off:+.2f} C",
            "------------------------------",
            "IGUALAR AL PATRON:",
            f"GIRAR          {cruda+off:.2f} C",
            "CLIC=OK"]


def resultados(c, L=136, tau=1176, K=0.878):
    c.setFont("6x10")
    c.drawStr(0, 8, "RESULTADOS")
    c.drawHLine(0, 11, 128)
    c.setFont("7x13")
    c.drawStr(2, 26, f"L   {L:.0f} s")
    c.drawStr(2, 39, f"TAU {tau:.0f} s")
    c.drawStr(2, 52, f"K   {K:.3f}")
    c.setFont("5x8")
    kp = 0.9 * tau / (K * L)
    c.drawStr(0, 62, f"KP {kp:.1f}  KI {kp/(3.33*L):.4f}")
    return ["RESULTADOS",
            "------------------------------",
            f"L   {L:.0f} s",
            f"TAU {tau:.0f} s",
            f"K   {K:.3f}",
            f"KP {kp:.1f}  KI {kp/(3.33*L):.4f}"]


def _partir(t, ancho=25):
    """Parte por un espacio, como hace el firmware: cortar a los 25 exactos
    dejaria 'LIMITE 1' / '28 C', que obliga a reconstruir lo que se lee."""
    if len(t) <= ancho:
        return t, ""
    corte = ancho
    while corte > 10 and t[corte] != " ":
        corte -= 1
    if corte <= 10:
        corte = ancho
    return t[:corte], t[corte + 1:corte + 1 + ancho]


def falla(c, motivo="T=128.4 C SUPERA LIMITE 128 C"):
    c.setFont("7x13")
    c.drawBox(0, 0, 128, 15)
    c.setDrawColor(0)
    c.drawStr(3, 12, "*** FALLA ***")
    c.setDrawColor(1)
    c.setFont("5x8")
    l1, l2 = _partir(motivo)
    c.drawStr(0, 28, l1)
    if l2:
        c.drawStr(0, 38, l2)
    c.drawHLine(0, 44, 128)
    c.drawStr(0, 54, "SALIDA CORTADA")
    c.drawStr(0, 62, "DEJE PULSADO PARA SALIR")
    return ["### *** FALLA *** ###",
            "",
            l1, l2,
            "------------------------------",
            "SALIDA CORTADA",
            "DEJE PULSADO PARA SALIR"]


PANTALLAS = {
    "principal": principal, "menu": menu,
    "info": info, "calibracion": calibracion,
    "resultados": resultados, "falla": falla,
    "confirmar": confirmar, "purga": purga,
    "meseta": meseta, "cerrar": cerrar,
}
for _i in range(1, len(CHECKLIST) + 1):
    PANTALLAS[f"check{_i}"] = (lambda n: lambda c: checklist(c, n))(_i)


def main():
    pedidas = sys.argv[1:] or list(PANTALLAS)
    for nombre in pedidas:
        if nombre not in PANTALLAS:
            print(f"Pantalla desconocida: {nombre}")
            continue
        c = Lienzo()
        lineas = PANTALLAS[nombre](c)
        ruta = c.guardar(nombre, lineas)
        marco(nombre.upper(), lineas, c.avisos)
        print(f"  -> {ruta}")


if __name__ == "__main__":
    main()
