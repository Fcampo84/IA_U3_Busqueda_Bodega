# -*- coding: utf-8 -*-
"""
Agente robótico de bodega: búsqueda de rutas hacia lotes próximos a vencer (FEFO).

Inteligencia Artificial - Unidad 3: Problemas de búsqueda.
Autor: Francisco Mario Campo Torne.

El agente percibe el plano y el inventario de la bodega, selecciona los lotes con la
fecha de vencimiento más próxima (regla FEFO: primero en vencer, primero en salir) y
planifica la ruta desde su base de carga hasta el pasillo frente a cada lote.

Se comparan seis algoritmos: anchura (BFS), profundidad (DFS), profundidad iterativa
(IDS), costo uniforme (UCS), codiciosa (Greedy) y A*, sobre bodegas simuladas de cinco
tamaños, varias semillas y varios pedidos FEFO por bodega.

Uso:
    python busqueda_bodega.py              # experimento completo
    python busqueda_bodega.py --rapido     # revisión corta (menos de 1 minuto)

Requisitos: Python 3.9 o superior y matplotlib.
"""

import argparse
import csv
import heapq
import math
import os
import random
import statistics
import sys
import time
from collections import deque

import matplotlib
matplotlib.use("Agg")  # genera las figuras sin abrir ventanas
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

sys.setrecursionlimit(50000)

# --------------------------------------------------------------------------------------
# 1. Parámetros del experimento
# --------------------------------------------------------------------------------------
TAMANOS = {                      # nombre: (filas, columnas)
    "XS": (15, 20),
    "S": (25, 35),
    "M": (40, 60),
    "L": (60, 90),
    "XL": (80, 120),
}
SEMILLA_BASE = 2026
SEMILLAS = 40                    # bodegas distintas por tamaño
PEDIDOS = 10                     # lotes FEFO atendidos por bodega
PRESUPUESTO_IDS = 50_000         # máximo de nodos expandidos por IDS en un problema

COSTO_LIBRE = 1                  # costo de un paso en pasillo libre
COSTO_CONGESTION = 3             # costo de un paso en zona congestionada
PROB_ESTIBA = 0.04               # probabilidad de estiba temporal que bloquea un pasillo
PROB_CONGESTION = 0.12           # probabilidad de congestión en un pasillo transversal

LIBRE, ESTANTE, ESTIBA = 0, 1, 2
ACCIONES = [("Arriba", -1, 0), ("Derecha", 0, 1), ("Abajo", 1, 0), ("Izquierda", 0, -1)]
ALGORITMOS = ["BFS", "DFS", "IDS", "UCS", "Greedy", "A*"]
NOMBRES = {"BFS": "Anchura (BFS)", "DFS": "Profundidad (DFS)",
           "IDS": "Prof. iterativa (IDS)", "UCS": "Costo uniforme (UCS)",
           "Greedy": "Codiciosa (Greedy)", "A*": "A*"}


# --------------------------------------------------------------------------------------
# 2. Entorno: bodega simulada
# --------------------------------------------------------------------------------------
class Bodega:
    """Cuadrícula con estanterías, pasillos, estibas, zonas de congestión e inventario."""

    def __init__(self, filas, columnas, semilla):
        self.filas, self.columnas, self.semilla = filas, columnas, semilla
        rnd = random.Random(semilla)
        self.mapa = [[LIBRE] * columnas for _ in range(filas)]
        self.costo = [[COSTO_LIBRE] * columnas for _ in range(filas)]

        # Estanterías dobles separadas por pasillos verticales y cortadas por
        # pasillos transversales; las filas superiores e inferiores quedan libres.
        for f in range(2, filas - 4):
            if (f - 2) % 7 == 6:
                continue                               # pasillo transversal
            for c in range(1, columnas - 1):
                if c % 3 in (1, 2):
                    self.mapa[f][c] = ESTANTE

        # Zona de recepción y despacho (últimas 3 filas, tramo central): congestionada.
        for f in range(filas - 3, filas):
            for c in range(columnas // 4, 3 * columnas // 4):
                self.costo[f][c] = COSTO_CONGESTION

        # Congestión aleatoria en pasillos transversales (montacargas, reposición).
        for f in range(2, filas - 4):
            if (f - 2) % 7 == 6:
                for c in range(columnas):
                    if rnd.random() < PROB_CONGESTION:
                        self.costo[f][c] = COSTO_CONGESTION

        self.base = (filas - 1, 0)                     # base de carga del robot

        # Estibas temporales que bloquean celdas de pasillo.
        for f in range(filas):
            for c in range(columnas):
                if self.mapa[f][c] == LIBRE and (f, c) != self.base \
                        and rnd.random() < PROB_ESTIBA:
                    self.mapa[f][c] = ESTIBA

        # Inventario: cada celda de estantería contiene un lote con días para vencer.
        self.lotes = []
        for f in range(filas):
            for c in range(columnas):
                if self.mapa[f][c] == ESTANTE:
                    self.lotes.append({"lote": f"L{f:03d}{c:03d}", "ubicacion": (f, c),
                                       "dias_para_vencer": rnd.randint(1, 365)})

        self.alcanzables = self._alcanzables_desde_base()

    def transitable(self, celda):
        f, c = celda
        return 0 <= f < self.filas and 0 <= c < self.columnas and self.mapa[f][c] == LIBRE

    def sucesores(self, celda):
        """Modelo de transición: Resultado(s, a) y costo del paso."""
        f, c = celda
        for accion, df, dc in ACCIONES:
            nueva = (f + df, c + dc)
            if self.transitable(nueva):
                yield accion, nueva, self.costo[nueva[0]][nueva[1]]

    def _alcanzables_desde_base(self):
        visto, cola = {self.base}, deque([self.base])
        while cola:
            actual = cola.popleft()
            for _, sig, _ in self.sucesores(actual):
                if sig not in visto:
                    visto.add(sig)
                    cola.append(sig)
        return visto

    def pedidos_fefo(self, cantidad):
        """Percepción del inventario y regla FEFO: los lotes más próximos a vencer que
        tienen una celda de pasillo alcanzable frente a la estantería."""
        pedidos, objetivos = [], set()
        for lote in sorted(self.lotes, key=lambda x: (x["dias_para_vencer"], x["lote"])):
            f, c = lote["ubicacion"]
            for _, df, dc in ACCIONES:
                frente = (f + df, c + dc)
                if frente in self.alcanzables and frente != self.base \
                        and frente not in objetivos:
                    pedidos.append((lote, frente))
                    objetivos.add(frente)
                    break
            if len(pedidos) == cantidad:
                break
        return pedidos


def heuristica(celda, objetivo):
    """Distancia Manhattan multiplicada por el costo mínimo de un paso (admisible)."""
    return (abs(celda[0] - objetivo[0]) + abs(celda[1] - objetivo[1])) * COSTO_LIBRE


# --------------------------------------------------------------------------------------
# 3. Algoritmos de búsqueda
#    Cada uno devuelve: ruta, costo, pasos, nodos expandidos, frontera máxima,
#    memoria máxima y, opcionalmente, las celdas exploradas (para las figuras).
# --------------------------------------------------------------------------------------
def reconstruir(padres, objetivo):
    ruta, actual = [], objetivo
    while actual is not None:
        ruta.append(actual)
        actual = padres[actual]
    return ruta[::-1]


def resultado(bodega, ruta, expandidos, frontera_max, memoria_max, explorados):
    return {
        "ruta": ruta,
        "costo": sum(bodega.costo[f][c] for f, c in ruta[1:]) if ruta else None,
        "pasos": len(ruta) - 1 if ruta else None,
        "expandidos": expandidos,
        "frontera_max": frontera_max,
        "memoria_max": memoria_max,
        "explorados": explorados,
    }


def bfs(bodega, inicio, objetivo, registrar=False):
    """Anchura: cola FIFO; óptima en número de pasos, no en costo."""
    frontera, padres = deque([inicio]), {inicio: None}
    expandidos, frontera_max, explorados = 0, 1, []
    while frontera:
        actual = frontera.popleft()
        expandidos += 1
        if registrar:
            explorados.append(actual)
        if actual == objetivo:
            return resultado(bodega, reconstruir(padres, actual), expandidos,
                             frontera_max, frontera_max + len(padres), explorados)
        for _, sig, _ in bodega.sucesores(actual):
            if sig not in padres:
                padres[sig] = actual
                frontera.append(sig)
        frontera_max = max(frontera_max, len(frontera))
    return resultado(bodega, None, expandidos, frontera_max, len(padres), explorados)


def dfs(bodega, inicio, objetivo, registrar=False):
    """Profundidad: pila LIFO con control de visitados; no garantiza optimalidad."""
    frontera, padres = [inicio], {inicio: None}
    expandidos, frontera_max, explorados = 0, 1, []
    while frontera:
        actual = frontera.pop()
        expandidos += 1
        if registrar:
            explorados.append(actual)
        if actual == objetivo:
            return resultado(bodega, reconstruir(padres, actual), expandidos,
                             frontera_max, frontera_max + len(padres), explorados)
        for _, sig, _ in reversed(list(bodega.sucesores(actual))):
            if sig not in padres:
                padres[sig] = actual
                frontera.append(sig)
        frontera_max = max(frontera_max, len(frontera))
    return resultado(bodega, None, expandidos, frontera_max, len(padres), explorados)


class PresupuestoAgotado(Exception):
    """Se lanza cuando IDS supera el máximo de nodos permitido."""


def ids(bodega, inicio, objetivo, registrar=False):
    """Profundidad iterativa: búsquedas en profundidad limitada con límite creciente.
    Cada iteración guarda la menor profundidad con que alcanzó cada celda para no
    recorrer de nuevo caminos más largos. Si supera PRESUPUESTO_IDS nodos, se detiene y
    la corrida se registra como no completada."""
    contador = {"expandidos": 0, "pila_max": 0, "memo_max": 0}
    explorados = []

    def limitada(celda, profundidad, limite, camino, mejor):
        contador["expandidos"] += 1
        if contador["expandidos"] > PRESUPUESTO_IDS:
            raise PresupuestoAgotado
        contador["pila_max"] = max(contador["pila_max"], len(camino))
        if registrar:
            explorados.append(celda)
        if celda == objetivo:
            return list(camino)
        if profundidad == limite:
            return None
        for _, sig, _ in bodega.sucesores(celda):
            if mejor.get(sig, math.inf) <= profundidad + 1:
                continue
            mejor[sig] = profundidad + 1
            camino.append(sig)
            encontrada = limitada(sig, profundidad + 1, limite, camino, mejor)
            if encontrada:
                return encontrada
            camino.pop()
        return None

    try:
        for limite in range(bodega.filas * bodega.columnas):
            mejor = {inicio: 0}
            ruta = limitada(inicio, 0, limite, [inicio], mejor)
            contador["memo_max"] = max(contador["memo_max"], len(mejor))
            if ruta:
                return resultado(bodega, ruta, contador["expandidos"], contador["pila_max"],
                                 contador["pila_max"] + contador["memo_max"], explorados)
    except PresupuestoAgotado:
        contador["expandidos"] = PRESUPUESTO_IDS
    return resultado(bodega, None, contador["expandidos"], contador["pila_max"],
                     contador["pila_max"] + contador["memo_max"], explorados)


def _primero_el_mejor(bodega, inicio, objetivo, prioridad, registrar):
    """Esquema común de cola de prioridad para UCS, Greedy y A*."""
    desempate = 0
    frontera = [(prioridad(0, inicio), desempate, 0, inicio)]
    padres, mejor_g, cerrados = {inicio: None}, {inicio: 0}, set()
    expandidos, frontera_max, explorados = 0, 1, []
    while frontera:
        _, _, g, actual = heapq.heappop(frontera)
        if actual in cerrados:
            continue
        cerrados.add(actual)
        expandidos += 1
        if registrar:
            explorados.append(actual)
        if actual == objetivo:
            return resultado(bodega, reconstruir(padres, actual), expandidos, frontera_max,
                             frontera_max + len(cerrados) + len(mejor_g), explorados)
        for _, sig, paso in bodega.sucesores(actual):
            nuevo_g = g + paso
            if sig not in cerrados and nuevo_g < mejor_g.get(sig, math.inf):
                mejor_g[sig], padres[sig] = nuevo_g, actual
                desempate += 1
                heapq.heappush(frontera, (prioridad(nuevo_g, sig), desempate, nuevo_g, sig))
        frontera_max = max(frontera_max, len(frontera))
    return resultado(bodega, None, expandidos, frontera_max,
                     frontera_max + len(cerrados) + len(mejor_g), explorados)


def ucs(bodega, inicio, objetivo, registrar=False):
    """Costo uniforme: f(n) = g(n); óptima en costo."""
    return _primero_el_mejor(bodega, inicio, objetivo, lambda g, n: g, registrar)


def greedy(bodega, inicio, objetivo, registrar=False):
    """Codiciosa: f(n) = h(n); rápida, pero no garantiza optimalidad."""
    return _primero_el_mejor(bodega, inicio, objetivo,
                             lambda g, n: heuristica(n, objetivo), registrar)


def a_estrella(bodega, inicio, objetivo, registrar=False):
    """A*: f(n) = g(n) + h(n); óptima con heurística admisible y consistente."""
    return _primero_el_mejor(bodega, inicio, objetivo,
                             lambda g, n: g + heuristica(n, objetivo), registrar)


FUNCIONES = {"BFS": bfs, "DFS": dfs, "IDS": ids, "UCS": ucs, "Greedy": greedy, "A*": a_estrella}


# --------------------------------------------------------------------------------------
# 4. Verificación formal de la heurística (admisibilidad y consistencia)
# --------------------------------------------------------------------------------------
def verificar_heuristica(bodega, objetivo):
    """Calcula con Dijkstra inverso el costo real mínimo h*(n) de cada celda hasta el
    objetivo y comprueba h(n) <= h*(n) (admisible) y h(n) <= c(n, m) + h(m) (consistente)."""
    real, cola = {objetivo: 0}, [(0, objetivo)]
    while cola:
        d, v = heapq.heappop(cola)
        if d > real[v]:
            continue
        for _, u, _ in bodega.sucesores(v):
            nd = d + bodega.costo[v[0]][v[1]]
            if nd < real.get(u, math.inf):
                real[u] = nd
                heapq.heappush(cola, (nd, u))
    admisible = all(heuristica(n, objetivo) <= real[n] for n in real)
    consistente = all(heuristica(n, objetivo) <= paso + heuristica(m, objetivo)
                      for n in real for _, m, paso in bodega.sucesores(n))
    return admisible, consistente, len(real)


# --------------------------------------------------------------------------------------
# 5. Experimento
# --------------------------------------------------------------------------------------
def ejecutar_experimento(tamanos, semillas, pedidos):
    corridas, verificaciones, escenario = [], [], None
    total = len(tamanos) * semillas
    hechas, inicio = 0, time.perf_counter()
    for nombre, (filas, columnas) in tamanos.items():
        for s in range(semillas):
            semilla = SEMILLA_BASE + s
            bodega = Bodega(filas, columnas, semilla)
            for orden, (lote, objetivo) in enumerate(bodega.pedidos_fefo(pedidos), start=1):
                admisible, consistente, n = verificar_heuristica(bodega, objetivo)
                verificaciones.append({
                    "bodega": nombre, "semilla": semilla, "pedido": orden,
                    "celdas_evaluadas": n, "admisible": admisible, "consistente": consistente})
                es_ejemplo = (nombre == "S" and s == 0 and orden == 1)
                resultados = {}
                for alg in ALGORITMOS:
                    t0 = time.perf_counter()
                    r = FUNCIONES[alg](bodega, bodega.base, objetivo, registrar=es_ejemplo)
                    r["tiempo_ms"] = (time.perf_counter() - t0) * 1000
                    resultados[alg] = r
                optimo = resultados["UCS"]["costo"]
                for alg, r in resultados.items():
                    completo = r["ruta"] is not None
                    corridas.append({
                        "bodega": nombre, "filas": filas, "columnas": columnas,
                        "celdas": filas * columnas, "semilla": semilla, "pedido": orden,
                        "lote": lote["lote"], "dias_para_vencer": lote["dias_para_vencer"],
                        "algoritmo": alg, "completado": completo,
                        "pasos": r["pasos"], "costo": r["costo"], "costo_optimo": optimo,
                        "sobrecosto_pct": (round(100 * (r["costo"] - optimo) / optimo, 2)
                                           if completo else None),
                        "es_optima": (r["costo"] == optimo) if completo else None,
                        "nodos_expandidos": r["expandidos"],
                        "frontera_max": r["frontera_max"], "memoria_max": r["memoria_max"],
                        "tiempo_ms": round(r["tiempo_ms"], 3),
                    })
                if es_ejemplo:
                    escenario = (bodega, lote, objetivo, resultados)
            hechas += 1
            transcurrido = time.perf_counter() - inicio
            barra = "#" * int(30 * hechas / total)
            print(f"\r  [{barra:<30}] {hechas}/{total} bodegas | {transcurrido:5.0f} s",
                  end="", flush=True)
    print()
    return corridas, verificaciones, escenario


def media_ic(valores):
    """Media e intervalo de confianza al 95 % (aproximación normal)."""
    n = len(valores)
    if n == 0:
        return None, None, None
    m = statistics.fmean(valores)
    if n < 2:
        return m, m, m
    margen = 1.96 * statistics.stdev(valores) / math.sqrt(n)
    return m, m - margen, m + margen


def resumir(corridas, tamanos):
    resumen = []
    for nombre in tamanos:
        for alg in ALGORITMOS:
            filas = [c for c in corridas if c["bodega"] == nombre and c["algoritmo"] == alg]
            comp = [c for c in filas if c["completado"]]
            nodos, nodos_inf, nodos_sup = media_ic([c["nodos_expandidos"] for c in comp])
            sobre, sobre_inf, sobre_sup = media_ic([c["sobrecosto_pct"] for c in comp])
            r = lambda x, d=2: round(x, d) if x is not None else None
            resumen.append({
                "bodega": nombre, "celdas": filas[0]["celdas"], "algoritmo": alg,
                "problemas": len(filas),
                "completados_pct": r(100 * len(comp) / len(filas), 1),
                "nodos_prom": r(nodos, 1), "nodos_ic95_inf": r(nodos_inf, 1),
                "nodos_ic95_sup": r(nodos_sup, 1),
                "nodos_mediana": r(statistics.median([c["nodos_expandidos"] for c in comp]), 1)
                if comp else None,
                "memoria_prom": r(statistics.fmean([c["memoria_max"] for c in comp]), 1)
                if comp else None,
                "tiempo_ms_prom": r(statistics.fmean([c["tiempo_ms"] for c in comp]), 3)
                if comp else None,
                "costo_prom": r(statistics.fmean([c["costo"] for c in comp]), 1)
                if comp else None,
                "sobrecosto_prom": r(sobre), "sobrecosto_ic95_inf": r(sobre_inf),
                "sobrecosto_ic95_sup": r(sobre_sup),
                "sobrecosto_max": r(max(c["sobrecosto_pct"] for c in comp)) if comp else None,
                "optimas_pct": r(100 * sum(c["es_optima"] for c in comp) / len(comp), 1)
                if comp else None,
            })
    return resumen


def comparaciones_pareadas(corridas, tamanos):
    """Compara pares de algoritmos sobre exactamente los mismos problemas."""
    indice = {}
    for c in corridas:
        indice[(c["bodega"], c["semilla"], c["pedido"], c["algoritmo"])] = c
    pares = [("A*", "UCS"), ("A*", "BFS"), ("Greedy", "A*")]
    salida = []
    for nombre in tamanos:
        problemas = {(c["semilla"], c["pedido"]) for c in corridas if c["bodega"] == nombre}
        for a, b in pares:
            reduccion, menos, costo_mayor = [], 0, 0
            for sem, ped in problemas:
                ca, cb = indice[(nombre, sem, ped, a)], indice[(nombre, sem, ped, b)]
                reduccion.append(100 * (cb["nodos_expandidos"] - ca["nodos_expandidos"])
                                 / cb["nodos_expandidos"])
                menos += ca["nodos_expandidos"] < cb["nodos_expandidos"]
                costo_mayor += ca["costo"] > cb["costo"]
            m, inf, sup = media_ic(reduccion)
            salida.append({
                "bodega": nombre, "comparacion": f"{a} frente a {b}",
                "problemas": len(problemas),
                "reduccion_nodos_pct_prom": round(m, 2),
                "reduccion_ic95_inf": round(inf, 2), "reduccion_ic95_sup": round(sup, 2),
                "problemas_con_menos_nodos_pct": round(100 * menos / len(problemas), 1),
                "problemas_con_mayor_costo_pct": round(100 * costo_mayor / len(problemas), 1),
            })
    return salida


# --------------------------------------------------------------------------------------
# 6. Comportamiento del agente: traza percepción - planificación - acción
# --------------------------------------------------------------------------------------
def traza_agente(bodega, lote, objetivo, resultados):
    r = resultados["A*"]
    lineas = [
        "TRAZA DEL AGENTE ROBÓTICO DE BODEGA",
        "=" * 60,
        f"Bodega {bodega.filas}x{bodega.columnas} | semilla {bodega.semilla}",
        "",
        "1. PERCEPCIÓN",
        f"   Lotes en inventario: {len(bodega.lotes)}",
        f"   Posición inicial (base de carga): {bodega.base}",
        f"   Lote FEFO seleccionado: {lote['lote']} en estantería {lote['ubicacion']}, "
        f"vence en {lote['dias_para_vencer']} días",
        f"   Celda objetivo frente a la estantería: {objetivo}",
        "",
        "2. PLANIFICACIÓN (A*)",
        f"   Nodos expandidos: {r['expandidos']} | costo: {r['costo']} | pasos: {r['pasos']}",
        "",
        "3. ACCIÓN (actuadores)",
    ]
    for i in range(1, len(r["ruta"])):
        (f0, c0), (f1, c1) = r["ruta"][i - 1], r["ruta"][i]
        accion = next(a for a, df, dc in ACCIONES if (f0 + df, c0 + dc) == (f1, c1))
        zona = "congestión" if bodega.costo[f1][c1] == COSTO_CONGESTION else "pasillo libre"
        lineas.append(f"   Paso {i:3d}: {accion:<9} -> {(f1, c1)}  costo "
                      f"{bodega.costo[f1][c1]} ({zona})")
    lineas += ["", f"   Brazo de recolección: retira el lote {lote['lote']}.",
               "   Objetivo alcanzado. El agente regresa a la base para el despacho FEFO."]
    return "\n".join(lineas)


# --------------------------------------------------------------------------------------
# 7. Figuras (estilo de informe: sin títulos internos, 300 dpi, legibles en grises)
# --------------------------------------------------------------------------------------
plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 10, "axes.titlesize": 10,
    "axes.labelsize": 10, "legend.fontsize": 9, "axes.spines.top": False,
    "axes.spines.right": False, "axes.grid": True, "grid.alpha": 0.3,
    "grid.linewidth": 0.6, "savefig.dpi": 300, "savefig.bbox": "tight",
})
ESTILO = {  # color, marcador y línea: distinguibles en color y en escala de grises
    "BFS":    ("#1b4f8a", "o", "-"),
    "DFS":    ("#b2182b", "s", "--"),
    "IDS":    ("#7b3294", "^", "-."),
    "UCS":    ("#1a9850", "D", ":"),
    "Greedy": ("#e08214", "v", "--"),
    "A*":     ("#000000", "*", "-"),
}


def dibujar_bodega(ax, bodega):
    matriz = [[0] * bodega.columnas for _ in range(bodega.filas)]
    for f in range(bodega.filas):
        for c in range(bodega.columnas):
            if bodega.mapa[f][c] == ESTANTE:
                matriz[f][c] = 2
            elif bodega.mapa[f][c] == ESTIBA:
                matriz[f][c] = 3
            elif bodega.costo[f][c] == COSTO_CONGESTION:
                matriz[f][c] = 1
    cmap = ListedColormap(["#ffffff", "#f6dcae", "#b5b5b5", "#3d3d3d"])
    ax.imshow(matriz, cmap=cmap, vmin=0, vmax=3, interpolation="nearest")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)
    for lado in ax.spines.values():
        lado.set_visible(True)
        lado.set_linewidth(0.8)


def leyenda_plano():
    return [Patch(facecolor="#b5b5b5", edgecolor="#777777", label="Estantería"),
            Patch(facecolor="#3d3d3d", label="Estiba (obstáculo)"),
            Patch(facecolor="#f6dcae", edgecolor="#c9a46a",
                  label=f"Zona congestionada (costo {COSTO_CONGESTION})")]


def figura_rutas(bodega, lote, objetivo, resultados, archivo):
    fig, ax = plt.subplots(figsize=(7.2, 5.6))
    dibujar_bodega(ax, bodega)
    trazos = [("BFS", -0.2, "-"), ("Greedy", 0.0, "--"), ("A*", 0.2, "-")]
    manejadores = []
    for alg, d, linea in trazos:
        ruta = resultados[alg]["ruta"]
        color = ESTILO[alg][0] if alg != "A*" else "#00a0b0"
        ax.plot([c + d for _, c in ruta], [f + d for f, _ in ruta], color=color,
                linewidth=2.2, linestyle=linea, solid_capstyle="round")
        manejadores.append(Line2D([], [], color=color, linewidth=2.2, linestyle=linea,
                                  label=f"{NOMBRES[alg]}: costo {resultados[alg]['costo']}, "
                                        f"{resultados[alg]['pasos']} pasos"))
    ax.plot(bodega.base[1], bodega.base[0], "s", color="black", markersize=9)
    ax.plot(objetivo[1], objetivo[0], "*", color="#d7191c", markersize=17,
            markeredgecolor="white", markeredgewidth=0.8)
    manejadores += [Line2D([], [], marker="s", color="black", linestyle="", markersize=8,
                           label="Base de carga"),
                    Line2D([], [], marker="*", color="#d7191c", linestyle="", markersize=13,
                           label=f"Objetivo FEFO ({lote['lote']})")]
    ax.legend(handles=manejadores + leyenda_plano(), loc="upper center",
              bbox_to_anchor=(0.5, -0.02), ncol=2, frameon=False)
    fig.savefig(archivo)
    plt.close(fig)


def figura_exploracion(bodega, objetivo, resultados, archivo):
    fig, ejes = plt.subplots(1, 3, figsize=(10, 3.4))
    for ax, alg, letra in zip(ejes, ("BFS", "UCS", "A*"), "abc"):
        dibujar_bodega(ax, bodega)
        exp = set(resultados[alg]["explorados"])
        ax.scatter([c for _, c in exp], [f for f, _ in exp], s=5, color="#2c7fb8",
                   alpha=0.75, linewidths=0)
        ruta = resultados[alg]["ruta"]
        ax.plot([c for _, c in ruta], [f for f, _ in ruta], color="black", linewidth=1.5)
        ax.plot(objetivo[1], objetivo[0], "*", color="#d7191c", markersize=11)
        ax.set_xlabel(f"({letra}) {NOMBRES[alg]}\n{resultados[alg]['expandidos']} nodos "
                      f"expandidos, costo {resultados[alg]['costo']}")
    fig.savefig(archivo)
    plt.close(fig)


def figura_escalabilidad(resumen, tamanos, archivo):
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    for alg in ALGORITMOS:
        filas = [r for r in resumen if r["algoritmo"] == alg and r["nodos_prom"] is not None]
        x = [r["celdas"] for r in filas]
        color, marca, linea = ESTILO[alg]
        etiqueta = NOMBRES[alg] + (" (solo problemas completados)" if alg == "IDS" else "")
        ax.plot(x, [r["nodos_prom"] for r in filas], color=color, marker=marca,
                linestyle=linea, linewidth=1.6, markersize=6, label=etiqueta)
        ax.fill_between(x, [r["nodos_ic95_inf"] for r in filas],
                        [r["nodos_ic95_sup"] for r in filas], color=color, alpha=0.12,
                        linewidth=0)
        if alg == "IDS":
            for r in filas:
                ax.annotate(f"{r['completados_pct']:.0f} %", (r["celdas"], r["nodos_prom"]),
                            textcoords="offset points", xytext=(0, -15), ha="center",
                            fontsize=8, color=color)
    ax.axhline(PRESUPUESTO_IDS, color=ESTILO["IDS"][0], linewidth=0.9, linestyle=(0, (1, 2)))
    ax.annotate(f"Presupuesto de IDS: {PRESUPUESTO_IDS:,} nodos".replace(",", "."),
                (min(r["celdas"] for r in resumen), PRESUPUESTO_IDS),
                textcoords="offset points", xytext=(2, 4), fontsize=8,
                color=ESTILO["IDS"][0])
    ax.set_xscale("log")
    ax.set_yscale("log")
    celdas = [tamanos[n][0] * tamanos[n][1] for n in tamanos]
    ax.set_xticks(celdas)
    ax.set_xticklabels([f"{tamanos[n][0]}×{tamanos[n][1]}" for n in tamanos])
    ax.minorticks_off()
    ax.set_ylim(top=PRESUPUESTO_IDS * 3)
    ax.set_xlabel("Tamaño de la bodega (filas × columnas)")
    ax.set_ylabel("Nodos expandidos (media, escala logarítmica)")
    ax.legend(ncol=2, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.16))
    fig.savefig(archivo)
    plt.close(fig)


def figura_sobrecosto(corridas, tamanos, archivo):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4.2),
                                   gridspec_kw={"width_ratios": [3, 1]})
    nombres = list(tamanos)
    algs = ["BFS", "IDS", "Greedy"]
    ancho = 0.24
    for i, alg in enumerate(algs):
        datos = [[c["sobrecosto_pct"] for c in corridas
                  if c["bodega"] == n and c["algoritmo"] == alg and c["completado"]] or [0]
                 for n in nombres]
        pos = [j + (i - 1) * ancho for j in range(len(nombres))]
        color = ESTILO[alg][0]
        ax1.boxplot(datos, positions=pos, widths=ancho * 0.85, patch_artist=True,
                    showfliers=True, flierprops={"marker": ".", "markersize": 3,
                                                 "markerfacecolor": color, "alpha": 0.5},
                    medianprops={"color": "black"},
                    boxprops={"facecolor": color, "alpha": 0.55})
    ax1.set_xticks(range(len(nombres)))
    ax1.set_xticklabels([f"{tamanos[n][0]}×{tamanos[n][1]}" for n in nombres])
    ax1.set_xlabel("(a) Tamaño de la bodega")
    ax1.set_ylabel("Sobrecosto frente a la ruta óptima (%)")
    ax1.legend(handles=[Patch(facecolor=ESTILO[a][0], alpha=0.55, label=NOMBRES[a])
                        for a in algs], frameon=False, loc="upper left")
    ax1.text(0.99, 0.97, "UCS y A*: 0 % en todos los problemas", transform=ax1.transAxes,
             ha="right", va="top", fontsize=9)
    datos_dfs = [[c["sobrecosto_pct"] for c in corridas
                  if c["bodega"] == n and c["algoritmo"] == "DFS"] for n in nombres]
    ax2.boxplot(datos_dfs, patch_artist=True, widths=0.6,
                flierprops={"marker": ".", "markersize": 3, "alpha": 0.5},
                medianprops={"color": "black"},
                boxprops={"facecolor": ESTILO["DFS"][0], "alpha": 0.55})
    ax2.set_xticks(range(1, len(nombres) + 1))
    ax2.set_xticklabels([f"{tamanos[n][0]}×{tamanos[n][1]}" for n in nombres],
                        rotation=45, ha="right")
    ax2.set_ylabel("Sobrecosto (%)")
    ax2.set_xlabel("(b) Profundidad (DFS)")
    fig.tight_layout()
    fig.savefig(archivo)
    plt.close(fig)


def figura_equilibrio(resumen, archivo):
    """Eficiencia frente a calidad en la bodega más grande del experimento."""
    piso = 0.3                                   # posición gráfica del 0 % en escala log
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    desplazamientos = {"A*": (-10, 10), "UCS": (10, 10), "BFS": (10, 6), "DFS": (10, -4),
                       "IDS": (0, -16), "Greedy": (10, 6)}
    alineacion = {"A*": "right", "IDS": "center"}
    vertical = {"IDS": "top"}
    for alg in ALGORITMOS:
        r = next(x for x in resumen if x["algoritmo"] == alg)
        if r["nodos_prom"] is None:
            continue
        color, marca, _ = ESTILO[alg]
        y = max(r["sobrecosto_prom"], piso)
        ax.scatter(r["nodos_prom"], y, s=260 if alg == "A*" else 120, color=color, marker=marca,
                   edgecolor="white", linewidth=0.8, zorder=3)
        etiqueta = NOMBRES[alg]
        if r["completados_pct"] < 100:
            etiqueta += f"\n({r['completados_pct']:.0f} % de problemas completados)"
        ax.annotate(etiqueta, (r["nodos_prom"], y), textcoords="offset points",
                    xytext=desplazamientos[alg], fontsize=9,
                    ha=alineacion.get(alg, "left"), va=vertical.get(alg, "baseline"))
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_yticks([piso, 1, 10, 100, 1000])
    ax.set_yticklabels(["0", "1", "10", "100", "1000"])
    ax.minorticks_off()
    ax.set_ylim(piso / 1.8, 4000)
    xs = [r["nodos_prom"] for r in resumen if r["nodos_prom"] is not None]
    ax.set_xlim(min(xs) / 2, max(xs) * 2.5)
    ax.set_xlabel("Nodos expandidos (media, escala logarítmica): menor es más eficiente")
    ax.set_ylabel("Sobrecosto medio (%): menor es mejor")
    fig.savefig(archivo)
    plt.close(fig)


# --------------------------------------------------------------------------------------
# 8. Programa principal
# --------------------------------------------------------------------------------------
def guardar_csv(filas, archivo):
    with open(archivo, "w", newline="", encoding="utf-8-sig") as f:
        escritor = csv.DictWriter(f, fieldnames=list(filas[0].keys()), delimiter=";")
        escritor.writeheader()
        escritor.writerows(filas)


def main():
    parser = argparse.ArgumentParser(description="Agente de bodega FEFO con búsqueda.")
    parser.add_argument("--semillas", type=int, default=SEMILLAS)
    parser.add_argument("--pedidos", type=int, default=PEDIDOS)
    parser.add_argument("--rapido", action="store_true",
                        help="revisión corta: 3 tamaños, 5 semillas, 3 pedidos")
    parser.add_argument("--salida", default="resultados")
    args = parser.parse_args()
    tamanos = TAMANOS
    if args.rapido:
        tamanos = {k: TAMANOS[k] for k in ("XS", "S", "M")}
        args.semillas, args.pedidos = 5, 3
    os.makedirs(args.salida, exist_ok=True)

    problemas = len(tamanos) * args.semillas * args.pedidos
    print(f"Experimento: {len(tamanos)} tamaños x {args.semillas} bodegas x "
          f"{args.pedidos} pedidos = {problemas} problemas, "
          f"{problemas * len(ALGORITMOS)} corridas")
    inicio = time.perf_counter()
    corridas, verificaciones, escenario = ejecutar_experimento(tamanos, args.semillas,
                                                               args.pedidos)
    resumen = resumir(corridas, tamanos)
    pareadas = comparaciones_pareadas(corridas, tamanos)

    guardar_csv(corridas, os.path.join(args.salida, "01_corridas.csv"))
    guardar_csv(resumen, os.path.join(args.salida, "02_resumen.csv"))
    guardar_csv(verificaciones, os.path.join(args.salida, "03_verificacion_heuristica.csv"))
    guardar_csv(pareadas, os.path.join(args.salida, "05_comparaciones_pareadas.csv"))
    bodega, lote, objetivo, resultados = escenario
    with open(os.path.join(args.salida, "04_traza_agente.txt"), "w", encoding="utf-8") as f:
        f.write(traza_agente(bodega, lote, objetivo, resultados))

    print("Generando figuras...")
    figura_rutas(bodega, lote, objetivo, resultados,
                 os.path.join(args.salida, "fig1_rutas_bodega.png"))
    figura_exploracion(bodega, objetivo, resultados,
                       os.path.join(args.salida, "fig2_exploracion.png"))
    figura_escalabilidad(resumen, tamanos, os.path.join(args.salida, "fig3_escalabilidad.png"))
    figura_sobrecosto(corridas, tamanos, os.path.join(args.salida, "fig4_sobrecosto.png"))
    figura_equilibrio([r for r in resumen if r["bodega"] == list(tamanos)[-1]],
                      os.path.join(args.salida, "fig5_equilibrio.png"))

    completas = [c for c in corridas if c["completado"]]
    controles = {
        "Heurística admisible en todos los problemas":
            all(v["admisible"] for v in verificaciones),
        "Heurística consistente en todos los problemas":
            all(v["consistente"] for v in verificaciones),
        "BFS, DFS, UCS, Greedy y A* encontraron ruta en todos los problemas":
            all(c["completado"] for c in corridas if c["algoritmo"] != "IDS"),
        "A* óptimo en todos los problemas":
            all(c["es_optima"] for c in corridas if c["algoritmo"] == "A*"),
        "UCS óptimo en todos los problemas":
            all(c["es_optima"] for c in corridas if c["algoritmo"] == "UCS"),
        "IDS completado con el mismo número de pasos que BFS": all(
            c["pasos"] == next(x["pasos"] for x in completas if x["bodega"] == c["bodega"]
                               and x["semilla"] == c["semilla"] and x["pedido"] == c["pedido"]
                               and x["algoritmo"] == "BFS")
            for c in completas if c["algoritmo"] == "IDS"),
    }

    print(f"\n{'Bodega':<7}{'Algoritmo':<9}{'Compl.%':>8}{'Nodos':>10}{'IC95%':>20}"
          f"{'Sobrecosto%':>13}{'Máx%':>9}{'Óptima%':>9}")
    for r in resumen:
        if r["nodos_prom"] is None:
            print(f"{r['bodega']:<7}{r['algoritmo']:<9}{r['completados_pct']:>8.1f}"
                  f"{'sin rutas completadas':>40}")
            continue
        ic = f"[{r['nodos_ic95_inf']:.0f}, {r['nodos_ic95_sup']:.0f}]"
        print(f"{r['bodega']:<7}{r['algoritmo']:<9}{r['completados_pct']:>8.1f}"
              f"{r['nodos_prom']:>10.1f}{ic:>20}{r['sobrecosto_prom']:>13.2f}"
              f"{r['sobrecosto_max']:>9.1f}{r['optimas_pct']:>9.1f}")
    print("\nControles de calidad:")
    for nombre, ok in controles.items():
        print(f"  [{'OK' if ok else 'FALLA'}] {nombre}")
    print(f"\nCorridas: {len(corridas)} | Tiempo total: {time.perf_counter() - inicio:.0f} s")
    print(f"Resultados guardados en: {os.path.abspath(args.salida)}")
    if not all(controles.values()):
        sys.exit(1)


if __name__ == "__main__":
    main()
