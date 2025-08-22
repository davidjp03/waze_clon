from Node import Node
import heapq
import osmnx as ox
import networkx as nx
import geopy
from geopy.geocoders import Nominatim
from geopy.distance import geodesic
import plotly.express as px
import webbrowser
import os
import pandas as pd

from constants import HEURISTIC_OPTIMISTIC_SPEED_KMH
from utils import get_turn_penalty, get_traffic_factor, get_weather_factor, get_time_factor, get_osrm_duration


class Waze(Node):
    def __init__(self, G, start, goal):
        super().__init__(state=start, value=0, operators=[])
        self.G = G
        self.start = start
        self.goal = goal

        # 🔹 Cachear factores externos SOLO UNA VEZ
        start_coord = (self.G.nodes[start]['y'], self.G.nodes[start]['x'])
        self.traffic_factor = get_traffic_factor(*start_coord)
        self.weather_factor = get_weather_factor(*start_coord)
        self.time_factor = get_time_factor()

        # 🔹 Precalcular tiempos base de cada arista
        for u, v, data in self.G.edges(data=True):
            dist_m = data.get("length", 1)
            speed_kmh = data.get("speed_kph", 30)
            base_time_sec = (dist_m / 1000) / speed_kmh * 3600
            data["base_time_sec"] = base_time_sec

    def cost(self, from_node, to_node, prev_node=None):
        """
        Calcula el costo en segundos para ir de from_node a to_node,
        considerando velocidad de vía, giros y factores externos.
        """
        edge_data = self.G.edges[(from_node, to_node, 0)]
        base_time_sec = edge_data["base_time_sec"]

        # Penalización por giro
        turn_penalty = get_turn_penalty(self.G, from_node, to_node, prev_node)

        # 🔹 Usar los factores precalculados
        total_time = (base_time_sec + turn_penalty) * \
                     self.traffic_factor * self.weather_factor * self.time_factor
        return total_time

    def heuristic(self, node):
        """
        Estimación optimista del tiempo hasta el objetivo.
        """
        coord1 = (self.G.nodes[node]['y'], self.G.nodes[node]['x'])
        coord2 = (self.G.nodes[self.goal]['y'], self.G.nodes[self.goal]['x'])
        dist_m = geodesic(coord1, coord2).meters
        return (dist_m / 1000) / HEURISTIC_OPTIMISTIC_SPEED_KMH * 3600

    def astar(self):
        """
        Algoritmo A* optimizado con heapq.
        """
        open_heap = []  # priority queue (f_score, node)
        heapq.heappush(open_heap, (self.heuristic(self.start), self.start))

        came_from = {}
        g_score = {node: float("inf") for node in self.G.nodes}
        g_score[self.start] = 0

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current == self.goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.G.neighbors(current):
                prev_node = came_from.get(current, None)
                tentative_g = g_score[current] + self.cost(current, neighbor, prev_node)

                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor)
                    heapq.heappush(open_heap, (f_score, neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

def generate_df(route):
    node_start = []
    node_end = []
    X_to = []
    Y_to = []
    X_from = []
    Y_from = []
    length = []
    travel_time = []

    for u, v in zip(route[:-1], route[1:]):
        node_start.append(u)
        node_end.append(v)
        length.append(round(G.edges[(u, v, 0)]['length']))
        travel_time.append(round(G.edges[(u, v, 0)]['travel_time']))
        X_from.append(G.nodes[u]['x'])
        Y_from.append(G.nodes[u]['y'])
        X_to.append(G.nodes[v]['x'])
        Y_to.append(G.nodes[v]['y'])
        
    df = pd.DataFrame(list(zip(node_start, node_end, X_from, Y_from,  X_to, Y_to, length, travel_time)),
               columns =["node_start", "node_end", "X_from", "Y_from",  "X_to", "Y_to", "length", "travel_time"])

    return df


def show_route_map(df, start, end):
    # Agregar columna 'index' para usar en animation_frame
    df['index'] = df.index  

    fig = px.scatter_mapbox(
        df,
        lon="X_from",
        lat="Y_from",
        zoom=12,
        width=1000,
        height=600,
        animation_frame="index",
        mapbox_style='open-street-map'
    )

    # Estilo para los puntos de la ruta
    fig.data[0].marker = dict(size=12, color="black")

    # Punto de inicio
    start_df = pd.DataFrame([{"X_from": start[1], "Y_from": start[0]}])
    fig.add_trace(px.scatter_mapbox(start_df, lon="X_from", lat="Y_from").data[0])
    fig.data[-1].marker = dict(size=15, color="red")

    # Punto de fin
    end_df = pd.DataFrame([{"X_from": end[1], "Y_from": end[0]}])
    fig.add_trace(px.scatter_mapbox(end_df, lon="X_from", lat="Y_from").data[0])
    fig.data[-1].marker = dict(size=15, color="green")

    # Línea de la ruta
    fig.add_trace(px.line_mapbox(df, lon="X_from", lat="Y_from").data[0])

    # Guardar HTML
    html_file = "ruta.html"
    fig.write_html(html_file)

    # Abrir automáticamente en el navegador
    webbrowser.open('file://' + os.path.realpath(html_file))


if __name__ == "__main__":
    locator = Nominatim(user_agent='myGeocoder')
    # Punto inicial y final (puedes cambiarlos aquí)
    location_start = locator.geocode('Universidad EIA, Envigado, Colombia')
    location_end=locator.geocode('Parque El Salado, Envigado, Colombia')
    start = (location_start.latitude, location_start.longitude)
    end = (location_end.latitude, location_end.longitude)      
    print(f"Inicio: {start}, Fin: {end}")
    # Descargar el grafo de calles
    G = ox.graph_from_place('Envigado, Antioquia, Colombia', network_type='drive')
    hwy_speeds = {'residential': 30,
                'secondary': 90,
                'tertiary': 120}
    G = ox.add_edge_speeds(G, hwy_speeds=hwy_speeds)
    G = ox.add_edge_travel_times(G)
    # Convertir nodos a coordenadas
    start_node = ox.distance.nearest_nodes(G, start[1], start[0])
    end_node = ox.distance.nearest_nodes(G, end[1], end[0])

    # Encontrar la ruta más corta con A*
    waze = Waze(G, start=start_node, goal=end_node)
    route = waze.astar()
    # Convertir nodos de la ruta a coordenadas
    df = generate_df(route)
    # Mostrar el mapa automáticamente
    show_route_map(df,start, end)
    #ox.plot_graph_route(G, route, route_linewidth=6, node_size=0, bgcolor='k',figsize=(20, 20))