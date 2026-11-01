# Agente robótico de bodega: búsqueda de rutas FEFO

Inteligencia Artificial - Unidad 3: Problemas de búsqueda.
Autor: Francisco Mario Campo Torne.

## Descripción

Un robot de bodega percibe el plano y el inventario, selecciona los lotes más próximos
a vencer (regla FEFO: primero en vencer, primero en salir) y planifica la ruta desde su
base de carga hasta el pasillo frente a cada lote. La bodega es una cuadrícula simulada
con estanterías, estibas que bloquean pasillos y zonas congestionadas de mayor costo.

Se comparan seis algoritmos: anchura (BFS), profundidad (DFS), profundidad iterativa
(IDS), costo uniforme (UCS), codiciosa (Greedy) y A*.

| Elemento | Definición |
|---|---|
| Estado | Celda (fila, columna) del robot |
| Estado inicial | Base de carga, esquina inferior izquierda |
| Acciones | Arriba, Derecha, Abajo, Izquierda, si la celda es pasillo libre |
| Objetivo | Celda de pasillo frente al lote FEFO |
| Costo | 1 por paso en pasillo libre; 3 en zona congestionada |
| Heurística | Distancia Manhattan × costo mínimo de un paso (admisible y consistente) |

## Diseño del experimento

| Parámetro | Valor |
|---|---|
| Tamaños de bodega | 15×20, 25×35, 40×60, 60×90 y 80×120 celdas |
| Bodegas por tamaño | 40 (semillas 2026 a 2065) |
| Pedidos FEFO por bodega | 10 |
| Problemas de búsqueda | 2.000 |
| Corridas (problemas × 6 algoritmos) | 12.000 |
| Presupuesto de IDS | 50.000 nodos por problema; si lo supera, se registra como no completado |

Los datos son simulados con semillas fijas: los resultados son reproducibles y solo
varían los tiempos de ejecución, que dependen del equipo.

## Entorno virtual e instalación

Windows (CMD):

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

macOS o Linux:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Ejecución

```bash
python busqueda_bodega.py            # experimento completo (2 a 5 minutos)
python busqueda_bodega.py --rapido   # revisión corta (menos de 1 minuto)
```

## Salidas (carpeta `resultados/`)

| Archivo | Contenido |
|---|---|
| `01_corridas.csv` | Una fila por problema y algoritmo (12.000 filas) |
| `02_resumen.csv` | Medias, intervalos de confianza al 95 %, máximos y optimalidad por tamaño y algoritmo |
| `03_verificacion_heuristica.csv` | Admisibilidad y consistencia de la heurística en cada problema |
| `04_traza_agente.txt` | Percepción, planificación y acciones del agente paso a paso |
| `05_comparaciones_pareadas.csv` | A* frente a UCS, A* frente a BFS y Greedy frente a A* sobre los mismos problemas |
| `fig1_rutas_bodega.png` | Rutas de BFS, Greedy y A* sobre el plano |
| `fig2_exploracion.png` | Celdas exploradas por BFS, UCS y A* |
| `fig3_escalabilidad.png` | Nodos expandidos según el tamaño de la bodega, con IC 95 % |
| `fig4_sobrecosto.png` | Distribución del sobrecosto frente a la ruta óptima |
| `fig5_equilibrio.png` | Eficiencia frente a calidad en la bodega más grande |

Los CSV usan punto y coma como separador para abrirse directamente en Excel. Las figuras
se generan a 300 dpi, sin títulos internos, para insertarse en un informe con formato APA.

## Controles de calidad

Al terminar, el programa verifica seis condiciones y muestra `[OK]` o `[FALLA]`:
heurística admisible y consistente en todos los problemas, ruta encontrada por BFS, DFS,
UCS, Greedy y A*, optimalidad de A* y UCS, y mismo número de pasos de IDS y BFS en los
problemas que IDS completa. Si alguna falla, el programa termina con código de error 1.
