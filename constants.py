PENALTY_LEFT_TURN_SEC = 8.0         # giro a la izquierda ~ semáforo/conflictos
PENALTY_UTURN_SEC = 20.0            # U-turn fuerte
PENALTY_RIGHT_TURN_SEC = 3.0        # giro a la derecha
PENALTY_STRAIGHT_SEC = 0.5          # leve pérdida por cruce recto
PENALTY_DEFAULT_SEC = 4.0

# Límites de factores
TRAFFIC_FACTOR_MIN, TRAFFIC_FACTOR_MAX = 0.6, 1.8   # <1 lento, >1 rápido
WEATHER_FACTOR_MIN, WEATHER_FACTOR_MAX = 1.0, 1.5   # >=1 (peor clima = más lento)
TIME_FACTOR_MIN, TIME_FACTOR_MAX = 0.8, 1.3         # horas valle vs pico

# Velocidad optimista (km/h) para la heurística (admisible): el mejor caso razonable
HEURISTIC_OPTIMISTIC_SPEED_KMH = 90.0  # vías rápidas urbanas/arterias sin congestión
