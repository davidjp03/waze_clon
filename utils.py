import math
from datetime import datetime
from api_manager import APIManager
from constants import (
    PENALTY_LEFT_TURN_SEC, PENALTY_UTURN_SEC,
    PENALTY_RIGHT_TURN_SEC, PENALTY_STRAIGHT_SEC, PENALTY_DEFAULT_SEC,
    TRAFFIC_FACTOR_MIN, TRAFFIC_FACTOR_MAX,
    WEATHER_FACTOR_MIN, WEATHER_FACTOR_MAX,
    TIME_FACTOR_MIN, TIME_FACTOR_MAX
)

def get_turn_penalty(G, from_node, to_node, prev_node):
    if prev_node is None:
        return 0  # Sin penalidad en el primer movimiento

    # Vector desde prev_node a from_node
    x1, y1 = G.nodes[from_node]['x'] - G.nodes[prev_node]['x'], G.nodes[from_node]['y'] - G.nodes[prev_node]['y']
    # Vector desde from_node a to_node
    x2, y2 = G.nodes[to_node]['x'] - G.nodes[from_node]['x'], G.nodes[to_node]['y'] - G.nodes[from_node]['y']

    dot = x1 * x2 + y1 * y2
    mag1 = math.hypot(x1, y1)
    mag2 = math.hypot(x2, y2)

    if mag1 == 0 or mag2 == 0:
        return 0

    cos_angle = dot / (mag1 * mag2)
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle = math.degrees(math.acos(cos_angle))

    # Clasificar el giro
    if angle < 30:
        return PENALTY_STRAIGHT_SEC
    elif 30 <= angle < 100:
        return PENALTY_RIGHT_TURN_SEC
    elif 100 <= angle < 170:
        return PENALTY_LEFT_TURN_SEC
    elif angle >= 170:
        return PENALTY_UTURN_SEC
    else:
        return PENALTY_DEFAULT_SEC

def get_traffic_factor(lat, lon):
    """
    Obtiene un factor de tráfico en función de la velocidad actual vs libre flujo.
    """
    traffic_data = APIManager.get_traffic_data(lat, lon)
    if traffic_data and 'flowSegmentData' in traffic_data:
        speed = traffic_data['flowSegmentData']['currentSpeed']
        free_flow = traffic_data['flowSegmentData']['freeFlowSpeed']
        factor = free_flow / max(speed, 1)
        return max(TRAFFIC_FACTOR_MIN, min(factor, TRAFFIC_FACTOR_MAX))
    return 1.0

def get_weather_factor(lat, lon):
    """
    Devuelve un factor según el clima en la ubicación.
    """
    weather = APIManager.get_weather(lat, lon)
    if weather and 'weather' in weather:
        condition = weather['weather'][0]['main'].lower()
        if 'rain' in condition or 'storm' in condition:
            return WEATHER_FACTOR_MAX
        elif 'cloud' in condition:
            return 1.1
    return WEATHER_FACTOR_MIN

def get_time_factor():
    """
    Devuelve un factor según la hora del día (hora pico vs valle).
    """
    hour = datetime.now().hour
    if 6 <= hour <= 9 or 17 <= hour <= 20:
        return TIME_FACTOR_MAX  # hora pico
    return TIME_FACTOR_MIN
