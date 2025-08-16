# api_manager.py
import os
import requests
from dotenv import load_dotenv

# Cargar variables de entorno del archivo .env
load_dotenv()

# Leer las API keys desde .env
OSRM_BASE_URL = os.getenv("OSRM_BASE_URL", "http://router.project-osrm.org")
WEATHER_API_KEY = os.getenv("OPENWEATHER_API_KEY")
TRAFFIC_API_KEY = os.getenv("TOM_TOM_API_KEY")

class APIManager:
    """
    Clase para gestionar las llamadas a las APIs externas.
    """

    @staticmethod
    def get_osrm_route(coord1, coord2, profile="driving"):
        """
        Obtiene una ruta entre dos coordenadas usando OSRM.
        coord1 y coord2 deben ser (lat, lon)
        """
        url = f"{OSRM_BASE_URL}/route/v1/{profile}/{coord1[1]},{coord1[0]};{coord2[1]},{coord2[0]}?overview=false"
        try:
            response = requests.get(url)
            response.raise_for_status()
            return response.json()
        except requests.RequestException as e:
            print(f"Error consultando OSRM: {e}")
            return None

    @staticmethod
    def get_weather(lat, lon):
        """
        Consulta clima usando OpenWeather API.
        """
        url = f"http://api.openweathermap.org/data/2.5/weather?lat={lat}&lon={lon}&appid={WEATHER_API_KEY}&units=metric"
        try:
            response = requests.get(url)
            response.raise_for_status()
            return response.json()
        except requests.RequestException as e:
            print(f"Error consultando clima: {e}")
            return None

    @staticmethod
    def get_traffic_data(lat, lon):
        """
        Consulta información de tráfico (ejemplo: TomTom Traffic API).
        """
        url = f"https://api.tomtom.com/traffic/services/4/flowSegmentData/absolute/10/json?point={lat},{lon}&key={TRAFFIC_API_KEY}"
        try:
            response = requests.get(url)
            response.raise_for_status()
            return response.json()
        except requests.RequestException as e:
            print(f"Error consultando tráfico: {e}")
            return None
