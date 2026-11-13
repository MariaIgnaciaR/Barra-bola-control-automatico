# -*- coding: utf-8 -*-
"""
Actividad 1 — Simulador interactivo de la planta «bola sobre barra»
CBM-422 Ingeniería de Control Automático — Universidad de Valparaíso

Planta: pivote en una esquina, servo SG90 en la otra, biela–manivela,
esfera maciza rodando ENTRE DOS RIELES.

Modelo implementado (deducción en el informe):
    x¨ = β·sin(α)        con  β = g / (1 + I/(m·r_eff²))
    I  = (2/5)·m·R²                    (esfera maciza)
    r_eff = sqrt(R² − (sep/2)²)        (rodadura entre dos rieles)
    tan(α) = a·sin(θ−θ0) / (d − a·cos(θ−θ0))   (brazo del servo empuja
             la barra DIRECTAMENTE desde abajo, sin biela)
    Servo SG90: velocidad angular limitada a 600°/s (0.1 s/60°)

Ejecutar:   python actividad1_simulador_pygame.py
Controles:
  Flecha ↑ / ↓ : comando del servo (modo MANUAL)
  Flecha ← / → : referencia de posición (modo PID)
  M            : cambiar modo MANUAL / PID
  F            : mostrar/ocultar el diagrama de fuerzas
  ESPACIO      : aplicar escalón de prueba
  R            : soltar la esfera desde el pivote
  S            : guardar captura de pantalla
  ESC          : salir
"""

import math
import os
import sys
from collections import deque

if "--test" in sys.argv:                 # modo prueba sin ventana
    os.environ["SDL_VIDEODRIVER"] = "dummy"

# ============================================================================
# PARÁMETROS FÍSICOS  (¡MEDIR EN LA PLANTA REAL Y REEMPLAZAR!)
# ============================================================================
g   = 9.81                # gravedad [m/s²]
m   = 0.0041              # masa de la esfera [kg]  (4.1 g, medida)
R   = 0.005               # radio de la esfera [m]  (0.5 cm, medido)
sep = 0.007               # separación entre los dos rieles [m] (0.7 cm, medida)
L   = 0.115               # largo útil de la barra [m] (11.5 cm, medido)
a   = 0.020               # largo del brazo (manivela) del servo [m] (2 cm, medido)
d   = 0.125               # distancia horizontal pivote → eje del servo [m] (12.5 cm, medida)
THETA0 = math.radians(90) # ángulo del servo con barra horizontal   ← IDENTIFICAR (Act. 2)
MU_ROCE = 0.02            # coeficiente de roce rodante esfera-grafito
                          # ← IDENTIFICAR experimentalmente (Act. 5):
                          #   μ = tan(α_crítico)  o  ajuste con tiempos de viaje

# --------- Constantes deducidas del modelo ----------
r_eff = math.sqrt(max(R * R - (sep / 2) ** 2, 1e-8))   # radio efectivo de rodadura
I     = (2 / 5) * m * R * R                            # momento de inercia (esfera maciza)
beta  = g / (1 + I / (m * r_eff * r_eff))              # ≈ (5/7)·g si los rieles casi se tocan

# --------- Servo SG90 ----------
OMEGA_MAX = math.radians(600.0)               # 0.1 s/60° → 600 °/s
ALFA_MAX  = math.asin(min(a / d, 0.98))       # inclinación máxima que entrega la biela
DTH_MAX   = math.radians(40)                  # recorrido útil del comando del servo

# --------- PID digital (anticipo de la Actividad 6) ----------
TS_PID = 0.03            # período de muestreo [s] (~30 fps de la cámara)
KP, KI, KD = 2.5, 0.8, 1.2     # ganancias iniciales (se sintonizan en la Act. 8)


# ============================================================================
# MODELO MATEMÁTICO
# ============================================================================
def alfa_desde_theta(theta):
    """Inclinación de la barra [rad] según el ángulo del servo.
    La punta del brazo (radio a) empuja la barra desde abajo:
    tan(α) = a·sin(Δ) / (d − a·cos(Δ)),  con Δ = θ − θ0."""
    D = theta - THETA0
    return math.atan2(a * math.sin(D), d - a * math.cos(D))


def theta_desde_alfa(alfa):
    """Ángulo del servo necesario para lograr la inclinación alfa [rad].
    De la condición de contacto:  a·sin(ψ+α) = d·sin(α)  →  ψ = θ − θ0."""
    s = max(-1.0, min(1.0, (d / a) * math.sin(alfa)))
    psi = math.asin(s) - alfa
    return THETA0 + psi


def aceleracion_esfera(alfa, v=0.0):
    """x¨ = β·(sin α − μ·cos α).  El roce rodante se opone al movimiento;
    si la esfera está quieta y |tan α| ≤ μ, el roce estático la sostiene."""
    a_g = beta * math.sin(alfa)
    a_f = beta * MU_ROCE * math.cos(alfa)
    if v == 0.0 and abs(a_g) <= a_f:
        return 0.0                       # zona muerta: no se mueve
    sentido = math.copysign(1.0, v) if v != 0.0 else math.copysign(1.0, a_g)
    return a_g - a_f * sentido


class Planta:
    """Estado físico: servo (θ) + barra (α) + esfera (x, v)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.theta_cmd = THETA0   # comando al servo [rad]
        self.theta     = THETA0   # ángulo real del servo [rad]
        self.alfa      = 0.0      # inclinación de la barra [rad]
        self.x         = 0.02     # posición de la esfera desde el pivote [m]
        self.v         = 0.0      # velocidad de la esfera [m/s]
        self.t         = 0.0

    def step(self, dt):
        # --- servo SG90: sigue el comando con velocidad angular limitada ---
        err  = self.theta_cmd - self.theta
        paso = max(-OMEGA_MAX * dt, min(OMEGA_MAX * dt, err))
        self.theta += paso
        # --- geometría biela–manivela ---
        self.alfa = alfa_desde_theta(self.theta)
        # --- esfera (integración semi-implícita de Euler) ---
        acc = aceleracion_esfera(self.alfa, self.v)
        v_new = self.v + acc * dt
        # si cambió de sentido y la gravedad no vence al roce: se detiene
        a_f = beta * MU_ROCE * math.cos(self.alfa)
        if self.v != 0.0 and v_new * self.v < 0 and abs(beta * math.sin(self.alfa)) <= a_f:
            v_new = 0.0
        self.v = v_new
        self.x += self.v * dt
        # --- topes físicos de la barra ---
        if self.x < 0.01:
            self.x, self.v = 0.01, 0.0
        if self.x > L - 0.01:
            self.x, self.v = L - 0.01, 0.0
        self.t += dt


class PID:
    """PID discreto con anti-windup por limitación (clamping)."""

    def __init__(self, kp=KP, ki=KI, kd=KD, ts=TS_PID, umax=ALFA_MAX):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.ts, self.umax = ts, umax
        self.integral = 0.0
        self.e_prev   = None
        self.reloj    = 0.0
        self.salida   = 0.0        # inclinación comandada α_cmd [rad]

    def reset(self):
        self.integral = 0.0
        self.e_prev   = None
        self.salida   = 0.0

    def actualizar(self, referencia, medicion, dt):
        self.reloj += dt
        if self.reloj < self.ts:          # corre a período fijo, como la cámara
            return self.salida
        Ts = self.reloj
        self.reloj = 0.0
        e = referencia - medicion
        # --- término derivativo (sobre el error) ---
        der = self.kd * (e - self.e_prev) / Ts if self.e_prev is not None else 0.0
        # --- candidato con término integral ---
        u = self.kp * e + self.ki * (self.integral + e * Ts) + der
        # --- anti-windup: solo acumula si el actuador NO está saturado ---
        if -self.umax < u < self.umax:
            self.integral += e * Ts
        self.salida = max(-self.umax, min(self.umax, u))
        self.e_prev = e
        return self.salida


class PasoMetricas:
    """Mide: tiempo de desplazamiento (10–90 %), velocidad máxima,
    tiempo de asentamiento (±2 mm), sobreimpulso y error estacionario."""

    BANDA = 0.002        # ±2 mm

    def __init__(self):
        self.activo = False

    def iniciar(self, t, x0, ref):
        self.activo = True
        self.t0, self.x0, self.ref = t, x0, ref
        self.delta   = ref - x0
        self.v_max   = 0.0
        self.t10 = self.t90 = None
        self.t_asent  = 0.0      # último instante FUERA de la banda (definición estricta)
        self.sobrepaso = 0.0
        self.ess_hist  = deque(maxlen=120)   # últimos ~2 s de |error|

    def actualizar(self, t, x, v):
        if not self.activo or self.delta == 0:
            return
        self.v_max = max(self.v_max, abs(v))
        avance = (x - self.x0) / self.delta
        if self.t10 is None and avance >= 0.10:
            self.t10 = t
        if self.t90 is None and avance >= 0.90:
            self.t90 = t
        # sobreimpulso: cuánto se pasa de la referencia en el sentido del paso
        self.sobrepaso = max(self.sobrepaso, (x - self.ref) * math.copysign(1, self.delta))
        # tiempo de asentamiento: instante a partir del cual NO vuelve a salir de la banda
        if abs(x - self.ref) > self.BANDA:
            self.t_asent = t - self.t0
        self.ess_hist.append(abs(x - self.ref))

    # ---- propiedades de lectura ----
    @property
    def t_desplazamiento(self):
        return None if (self.t10 is None or self.t90 is None) else self.t90 - self.t10

    @property
    def ess(self):
        return sum(self.ess_hist) / len(self.ess_hist) if self.ess_hist else 0.0

    @property
    def sobreimpulso_pct(self):
        return 100 * self.sobrepaso / abs(self.delta) if self.delta else 0.0


# ============================================================================
# PARTE GRÁFICA (pygame)
# ============================================================================
import pygame

ANCHO, ALTO = 1280, 780
ESCALA      = 5500.0                      # píxeles por metro (planta pequeña)
PIVOTE      = (ANCHO - 200, 380)          # pivote (tornillo) a la DERECHA,
                                          # servo a la IZQUIERDA, como la planta real

C_FONDO = (18, 22, 30);  C_BARRA = (70, 130, 180);  C_RIEL = (200, 200, 210)
C_BOLA  = (90, 220, 120); C_SERVO = (230, 140, 60);  C_BIELA = (160, 200, 240)
C_TEXTO = (235, 235, 235); C_REF   = (240, 240, 90); C_TRAZA = (120, 220, 130)
C_PESO  = (235, 70, 70);  C_NORMAL = (80, 200, 255); C_ROCE  = (250, 170, 40)


def pantalla_pt(p):
    return int(p[0]), int(p[1])


def flecha(surf, color, origen, vec, ancho=3):
    """Dibuja una flecha desde 'origen' con desplazamiento 'vec' (px)."""
    fin = (origen[0] + vec[0], origen[1] + vec[1])
    pygame.draw.line(surf, color, pantalla_pt(origen), pantalla_pt(fin), ancho)
    ang = math.atan2(vec[1], vec[0])
    for s in (-2.6, 2.6):
        punta = (fin[0] + 12 * math.cos(ang + s), fin[1] + 12 * math.sin(ang + s))
        pygame.draw.line(surf, color, pantalla_pt(fin), pantalla_pt(punta), ancho)


def dibujar(surf, fuente, fuente_g, pl, pid, modo, ref, metricas, historial, mostrar_fuerzas):
    surf.fill(C_FONDO)
    px, py = PIVOTE
    # +x: desde el pivote (derecha) hacia el servo (izquierda)
    u = (-math.cos(pl.alfa), math.sin(pl.alfa))      # a lo largo de la barra
    n = (-math.sin(pl.alfa), -math.cos(pl.alfa))     # perpendicular "arriba"

    # ---- piso ----
    gy = py + 130
    pygame.draw.line(surf, (90, 90, 100), (0, gy), (ANCHO, gy), 2)

    # ---- base fija: bloque rectangular de 15 cm + PARED TRASERA ----
    #      (la barra se atornilla a la pared por la derecha y el servo
    #       va incrustado en ella; NO hay paredes laterales)
    L_BASE = 0.15                                  # 15 cm (medido)
    x_base_d = px + 12                             # borde derecho de la base
    x_base_i = x_base_d - L_BASE * ESCALA          # borde izquierdo
    # pared trasera (va detrás de la barra, a lo largo de toda la base)
    pygame.draw.rect(surf, (216, 216, 221),
                     (x_base_i, py - 26, x_base_d - x_base_i, gy - (py - 26)))
    pygame.draw.line(surf, (150, 150, 155), (x_base_i, py - 26), (x_base_d, py - 26), 2)
    # bloque de la base que se apoya en la mesa (más oscuro, "cuadrado")
    pygame.draw.rect(surf, (190, 190, 196),
                     (x_base_i, gy - 44, x_base_d - x_base_i, 44))
    pygame.draw.rect(surf, (150, 150, 155),
                     (x_base_i, gy - 44, x_base_d - x_base_i, 44), 2)

    # ---- pivote = TORNILLO que atraviesa la barra y se atornilla a la pared ----
    pygame.draw.circle(surf, (175, 175, 180), (px, py), 11)
    pygame.draw.circle(surf, (90, 90, 95), (px, py), 11, 2)
    pygame.draw.circle(surf, (110, 110, 115), (px, py), 4)   # cabeza del tornillo

    # ---- barra (12 cm desde el tornillo-pivote hacia la izquierda) ----
    grosor = 14
    largo_barra = 0.12                             # 12 cm (medido)
    base_i = (px - n[0] * grosor, py - n[1] * grosor)
    base_f = (px + u[0] * largo_barra * ESCALA - n[0] * grosor,
              py + u[1] * largo_barra * ESCALA - n[1] * grosor)
    tope_f = (px + u[0] * largo_barra * ESCALA + n[0] * grosor,
              py + u[1] * largo_barra * ESCALA + n[1] * grosor)
    tope_i = (px + n[0] * grosor, py + n[1] * grosor)
    pygame.draw.polygon(surf, C_BARRA, [base_i, base_f, tope_f, tope_i])

    # tope físico al final de la zona útil
    t1 = (px + u[0] * L * ESCALA - n[0] * 4, py + u[1] * L * ESCALA - n[1] * 4)
    t2 = (px + u[0] * L * ESCALA + n[0] * (grosor + 14), py + u[1] * L * ESCALA + n[1] * (grosor + 14))
    pygame.draw.line(surf, (240, 90, 90), pantalla_pt(t1), pantalla_pt(t2), 5)

    # ---- los DOS RIELES sobre la barra ----
    for off in (3, 9):
        p1 = (px + n[0] * (grosor + off), py + n[1] * (grosor + off))
        p2 = (p1[0] + u[0] * L * ESCALA, p1[1] + u[1] * L * ESCALA)
        pygame.draw.line(surf, C_RIEL, pantalla_pt(p1), pantalla_pt(p2), 3)

    # ---- regla milimétrica (cada 20 mm) ----
    for k in range(0, int(L * 1000) + 1, 20):
        c = (px + u[0] * (k / 1000) * ESCALA, py + u[1] * (k / 1000) * ESCALA)
        a1 = (c[0] - n[0] * 6, c[1] - n[1] * 6)
        a2 = (c[0] - n[0] * 26, c[1] - n[1] * 26)
        pygame.draw.line(surf, (240, 240, 240), pantalla_pt(a1), pantalla_pt(a2), 1)
        txt = fuente.render(str(k), True, (200, 200, 200))
        surf.blit(txt, (a2[0] - 10, a2[1] + 4))

    # ---- servo SG90 ACOSTADO a la IZQUIERDA: cuerpo horizontal montado en
    #      la pared de la base; el brazo gira en el plano vertical y empuja
    #      la barra directamente desde abajo (como en la planta real) ----
    dth = pl.theta - THETA0
    x_c = d - a * math.cos(dth)                    # punto de contacto (se desliza
                                                   # sobre la barra al girar el brazo)
    cbx = px + u[0] * x_c * ESCALA - n[0] * grosor # punto en la cara inferior
    cby = py + u[1] * x_c * ESCALA - n[1] * grosor
    sx, sy = px - d * ESCALA, py + grosor          # eje del servo: a la izquierda,
                                                   # a la altura de la cara inferior

    # cuerpo del SG90 INCRUSTADO en la pared trasera (solo asoma; ~2 cm de alto)
    AZUL_SG90 = (48, 84, 190)
    pygame.draw.rect(surf, AZUL_SG90, (sx - 48, sy + 8, 100, 52), border_radius=3)
    pygame.draw.rect(surf, (30, 50, 120), (sx - 48, sy + 8, 100, 52), 2, border_radius=3)
    # etiqueta amarilla del SG90
    pygame.draw.rect(surf, (240, 220, 90), (sx - 36, sy + 20, 34, 16))

    # plato circular del horn + tornillo central (el eje)
    pygame.draw.circle(surf, (240, 240, 245), (int(sx), int(sy)), 9)
    pygame.draw.circle(surf, (120, 120, 125), (int(sx), int(sy)), 9, 1)
    pygame.draw.circle(surf, (90, 90, 95), (int(sx), int(sy)), 2)

    # brazo blanco: desde el eje hasta el punto de contacto bajo la barra
    pygame.draw.line(surf, (120, 120, 125), (sx, sy), (cbx, cby), 8)
    pygame.draw.line(surf, (245, 245, 248), (sx, sy), (cbx, cby), 5)
    pygame.draw.circle(surf, (250, 220, 120), (int(cbx), int(cby)), 5)  # contacto

    # ---- esfera ----
    cx = px + u[0] * pl.x * ESCALA + n[0] * (R * ESCALA + grosor + 6)
    cy = py + u[1] * pl.x * ESCALA + n[1] * (R * ESCALA + grosor + 6)
    pygame.draw.circle(surf, C_BOLA, (int(cx), int(cy)), int(R * ESCALA))
    pygame.draw.circle(surf, (20, 60, 30), (int(cx), int(cy)), int(R * ESCALA), 2)

    # ---- referencia (modo PID): marca sobre la barra ----
    if modo == "PID":
        rx = px + u[0] * ref * ESCALA + n[0] * 30
        ry = py + u[1] * ref * ESCALA + n[1] * 30
        pygame.draw.polygon(surf, C_REF, [(rx - 9, ry - 18), (rx + 9, ry - 18), (rx, ry)])

    # ---- diagrama de fuerzas sobre la esfera ----
    if mostrar_fuerzas:
        kN = 2500.0                         # px por Newton (esfera liviana: mg ≈ 0.04 N)
        acc = aceleracion_esfera(pl.alfa, pl.v)
        N   = m * g * math.cos(pl.alfa)     # normal
        fro = I / (r_eff ** 2) * acc        # roce (hace rodar la esfera)
        flecha(surf, C_PESO,   (cx, cy), (0, m * g * kN))                       # peso mg
        flecha(surf, C_NORMAL, (cx, cy), (n[0] * N * kN, n[1] * N * kN))        # normal N
        flecha(surf, C_ROCE,   (cx, cy), (-u[0] * fro * kN, -u[1] * fro * kN))  # roce f
        comp = m * g * math.sin(pl.alfa)                                      # mg·sinα
        flecha(surf, (250, 120, 200), (cx, cy), (u[0] * comp * kN, u[1] * comp * kN))
        for i, (nombre, col) in enumerate([("mg (peso)", C_PESO),
                                           ("N (normal)", C_NORMAL),
                                           ("f (roce)", C_ROCE),
                                           ("mg·sinα", (250, 120, 200))]):
            surf.blit(fuente.render(nombre, True, col), (ANCHO - 190, 60 + i * 22))

    # ---- panel de datos (HUD) ----
    lineas = [
        f"MODO: {modo}   (M cambia, F fuerzas, ESPACIO escalón, R reset)",
        f"t = {pl.t:6.2f} s",
        f"posición  x = {pl.x*1000:7.1f} mm",
        f"velocidad v = {pl.v*1000:7.1f} mm/s",
        f"barra  α = {math.degrees(pl.alfa):+6.2f}°",
        f"servo  θ = {math.degrees(pl.theta):6.1f}°",
        f"β = {beta:.3f} m/s²   r_eff = {r_eff*1000:.2f} mm   μ = {MU_ROCE}",
    ]
    if modo == "PID":
        lineas.append(f"referencia x* = {ref*1000:.1f} mm")
    for i, s in enumerate(lineas):
        surf.blit(fuente.render(s, True, C_TEXTO), (30, 20 + i * 22))

    # ---- panel de métricas del escalón ----
    if metricas.activo:
        me = [
            "── MÉTRICAS DEL ESCALÓN ──",
            f"v máx:           {metricas.v_max*1000:7.1f} mm/s",
            f"t desplaz.(10-90%): {('%6.3f' % metricas.t_desplazamiento) if metricas.t_desplazamiento else '  --- '} s",
            f"t asentamiento ±2mm:{('%6.3f' % metricas.t_asent) if metricas.t_asent else '  --- '} s",
            f"sobreimpulso:    {metricas.sobreimpulso_pct:6.1f} %",
            f"error estac.:    {metricas.ess*1000:6.2f} mm",
        ]
        for i, s in enumerate(me):
            surf.blit(fuente.render(s, True, (180, 220, 255)), (30, 210 + i * 22))

    # ---- gráfico de posición vs tiempo (tira continua) ----
    gx, gy, gw, gh = 420, ALTO - 190, ANCHO - 460, 150
    pygame.draw.rect(surf, (28, 32, 42), (gx, gy, gw, gh))
    pygame.draw.rect(surf, (100, 100, 110), (gx, gy, gw, gh), 1)
    surf.blit(fuente.render("x(t) [mm] — verde: medida, amarilla: referencia", True, C_TEXTO),
              (gx + 8, gy - 20))

    def mapear_y(valor_m):
        return gy + gh - (valor_m / L) * gh

    if len(historial) > 1:
        pts_x, pts_r = [], []
        for i, (xm, rm) in enumerate(historial):
            xx = gx + gw * i / (len(historial) - 1) if len(historial) > 1 else gx
            pts_x.append((xx, mapear_y(xm)))
            pts_r.append((xx, mapear_y(rm)))
        pygame.draw.lines(surf, C_REF, False, pts_r, 2)
        pygame.draw.lines(surf, C_TRAZA, False, pts_x, 2)

    surf.blit(fuente_g.render("SIMULADOR BOLA SOBRE BARRA — Actividad 1 (CBM-422)",
                              True, C_TEXTO), (420, ALTO - 32))


# ============================================================================
# PROGRAMA PRINCIPAL
# ============================================================================
def main():
    pygame.init()
    pantalla = pygame.display.set_mode((ANCHO, ALTO))
    pygame.display.set_caption("Simulador Bola sobre Barra — Actividad 1")
    reloj   = pygame.time.Clock()
    fuente  = pygame.font.SysFont("dejavusansmono", 17)
    fuente_g = pygame.font.SysFont("dejavusansmono", 22, bold=True)

    pl       = Planta()
    pid      = PID()
    metricas = PasoMetricas()
    historial = deque(maxlen=600)          # últimos ~10 s a 60 fps
    modo = "MANUAL"
    ref  = 0.03                            # referencia inicial 30 mm
    mostrar_fuerzas = True
    corriendo = True

    while corriendo:
        dt_frame = min(reloj.tick(60) / 1000.0, 0.05)

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                corriendo = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    corriendo = False
                elif ev.key == pygame.K_m:
                    modo = "PID" if modo == "MANUAL" else "MANUAL"
                    pid.reset()
                elif ev.key == pygame.K_f:
                    mostrar_fuerzas = not mostrar_fuerzas
                elif ev.key == pygame.K_r:
                    pl.reset(); pid.reset(); metricas.activo = False
                elif ev.key == pygame.K_s:
                    pygame.image.save(pantalla, "captura_simulador.png")
                    print("Captura guardada en captura_simulador.png")
                elif ev.key == pygame.K_SPACE:
                    # escalón de prueba: 30 ↔ 60 mm (PID) o ±10° (manual)
                    if modo == "PID":
                        ref = 0.060 if ref < 0.045 else 0.030
                        metricas.iniciar(pl.t, pl.x, ref)
                    else:
                        pl.theta_cmd = THETA0 + math.radians(10) \
                            if pl.theta_cmd <= THETA0 else THETA0 - math.radians(10)

        teclas = pygame.key.get_pressed()
        if modo == "MANUAL":
            if teclas[pygame.K_UP]:
                pl.theta_cmd = min(THETA0 + DTH_MAX, pl.theta_cmd + math.radians(60) * dt_frame)
            if teclas[pygame.K_DOWN]:
                pl.theta_cmd = max(THETA0 - DTH_MAX, pl.theta_cmd - math.radians(60) * dt_frame)
        else:
            if teclas[pygame.K_RIGHT]:
                ref = min(L - 0.01, ref + 0.020 * dt_frame)
            if teclas[pygame.K_LEFT]:
                ref = max(0.01, ref - 0.020 * dt_frame)
            alfa_cmd = pid.actualizar(ref, pl.x, dt_frame)
            pl.theta_cmd = theta_desde_alfa(alfa_cmd)

        # --- integración física con pasos de 1 ms ---
        n_sub = max(1, int(dt_frame / 0.001))
        for _ in range(n_sub):
            pl.step(dt_frame / n_sub)

        metricas.actualizar(pl.t, pl.x, pl.v)
        historial.append((pl.x, ref if modo == "PID" else 0.0))
        dibujar(pantalla, fuente, fuente_g, pl, pid, modo, ref,
                metricas, historial, mostrar_fuerzas)
        pygame.display.flip()

    pygame.quit()


# ============================================================================
# MODO PRUEBA (sin ventana): verificación numérica del modelo
# ============================================================================
def test():
    print(f"β = {beta:.4f} m/s²  (5/7·g = {5/7*g:.4f})   r_eff = {r_eff*1000:.2f} mm")
    print(f"α máx del mecanismo = {math.degrees(ALFA_MAX):.2f}°")

    # --- 1) Lazo abierto: comparación contra la solución analítica ---
    alfa = math.radians(3.0)
    x0, DX = 0.02, 0.08
    acc = aceleracion_esfera(alfa)
    t_teo = math.sqrt(2 * DX / acc)          # tiempo de desplazamiento teórico
    v_teo = acc * t_teo                      # velocidad máxima teórica
    pl = Planta(); pl.x = x0
    pl.theta = pl.theta_cmd = theta_desde_alfa(alfa)   # inclinación instantánea
    t_sim = v_sim = None
    while pl.t < 3.0:
        pl.step(0.001)
        if t_sim is None and pl.x >= x0 + DX:
            t_sim, v_sim = pl.t, pl.v
            break
    print(f"\n[LAZO ABIERTO α=3°] desplazamiento {DX*1000:.0f} mm desde {x0*1000:.0f} mm")
    print(f"  tiempo teórico:  {t_teo:.4f} s   |   simulado: {t_sim:.4f} s")
    print(f"  vel. máx teórica: {v_teo*1000:.1f} mm/s |   simulada: {v_sim*1000:.1f} mm/s")

    # --- 2) Lazo cerrado PID: escalón 30 → 60 mm ---
    pl = Planta(); pl.x = 0.030
    pid = PID(); met = PasoMetricas()
    ref = 0.060
    met.iniciar(0.0, pl.x, ref)
    while pl.t < 5.0:
        alfa_cmd = pid.actualizar(ref, pl.x, 0.001)
        pl.theta_cmd = theta_desde_alfa(alfa_cmd)
        pl.step(0.001)
        met.actualizar(pl.t, pl.x, pl.v)
    print(f"\n[LAZO CERRADO PID] escalón 30 → 60 mm")
    print(f"  t desplazamiento (10–90 %): {met.t_desplazamiento:.3f} s")
    print(f"  t asentamiento (±2 mm):     {met.t_asent:.3f} s")
    print(f"  sobreimpulso:               {met.sobreimpulso_pct:.1f} %")
    print(f"  velocidad máxima:           {met.v_max*1000:.1f} mm/s")
    print(f"  error estacionario:         {met.ess*1000:.2f} mm")

    # --- 3) render de un cuadro para verificar la parte gráfica ---
    pygame.init()
    pantalla = pygame.display.set_mode((ANCHO, ALTO))
    dibujar(pantalla, pygame.font.SysFont("dejavusansmono", 17),
            pygame.font.SysFont("dejavusansmono", 22, bold=True),
            pl, pid, "PID", ref, met, [(0.03, 0.06)] * 60, True)
    pygame.image.save(pantalla, "/mnt/agents/output/actividad1_captura_simulador.png")
    pygame.quit()
    print("\nCaptura de prueba guardada en actividad1_captura_simulador.png")


if __name__ == "__main__":
    if "--test" in sys.argv:
        test()
    else:
        main()