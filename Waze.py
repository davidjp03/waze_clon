from Node import Node
import osmnx as ox
import networkx as nx
import geopy
from geopy.geocoders import Nominatim
from geopy.distance import geodesic
import plotly.express as px
import webbrowser
import os
import pandas as pd

from geopy.geocoders import Nominatim
class Waze(Node):
    def __init__(self, G, start, goal):
        super().__init__(state=start, value=0, operators=[])
        self.G = G
        self.start = start
        self.goal = goal

    def getState(self, index):
        neighbors = list(self.G.neighbors(self.state))
        if 0 <= index < len(neighbors):
            return neighbors[index]
        return None

    def cost(self, from_node, to_node):
        # Costo = distancia geodésica entre dos nodos
        coord1 = (self.G.nodes[from_node]['y'], self.G.nodes[from_node]['x'])
        coord2 = (self.G.nodes[to_node]['y'], self.G.nodes[to_node]['x'])
        return geodesic(coord1, coord2).meters

    def heuristic(self, node):
        # Distancia estimada al objetivo
        coord1 = (self.G.nodes[node]['y'], self.G.nodes[node]['x'])
        coord2 = (self.G.nodes[self.goal]['y'], self.G.nodes[self.goal]['x'])
        return geodesic(coord1, coord2).meters

    def astar(self):
        open_set = {self.start}
        came_from = {}
        g_score = {node: float("inf") for node in self.G.nodes}
        g_score[self.start] = 0
        f_score = {node: float("inf") for node in self.G.nodes}
        f_score[self.start] = self.heuristic(self.start)

        while open_set:
            current = min(open_set, key=lambda node: f_score[node])
            if current == self.goal:
                return self.reconstruct_path(came_from, current)

            open_set.remove(current)
            for neighbor in self.G.neighbors(current):
                tentative_g_score = g_score[current] + self.cost(current, neighbor)
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor)
                    open_set.add(neighbor)

        return None  # No se encontró ruta

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
    location_start = locator.geocode('Sede Posgrados EIA, Envigado, Colombia')
    location_end=locator.geocode('Universidad EIA, Envigado, Colombia')
    start = (location_start.latitude, location_start.longitude)
    end = (location_end.latitude, location_end.longitude)      
    print(f"Inicio: {start}, Fin: {end}")
    # Descargar el grafo de calles
    G = ox.graph_from_place('Envigado, Antioquia, Colombia', network_type='drive')
    hwy_speeds = {'residential': 35,
                'secondary': 50,
                'tertiary': 60}
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