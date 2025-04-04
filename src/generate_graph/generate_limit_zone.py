import argparse
import numpy as np
import pandas as pd
from geopy.distance import geodesic


class CLI:
    def __init__(self):
        self.parser = argparse.ArgumentParser(
            description="Скрипт для создания csv файла с координатами точек в заданном радиусе"
        )
        self._setup_arguments()

    def _setup_arguments(self):
        """Настройка аргументов командной строки."""
        self.parser.add_argument("input", help="Входной файл")
        self.parser.add_argument("output", help="Выходной файл")
        self.parser.add_argument("center", type=int, help="Индекс центральной точки")
        self.parser.add_argument("radius", type=int, help="Радиус зоны в метрах")

    def parse_args(self):
        """Разбор аргументов и возврат результата."""
        return self.parser.parse_args()


class CSVGeoProcessor:
    def __init__(self, input_csv_path):
        """
        Инициализация процессора.

        :param input_csv_path: Путь к входному CSV-файлу с колонками index, longitude, latitude.
        """
        self.input_csv_path = input_csv_path
        self.data = None
        self.load_data()

    def load_data(self):
        """Загружает данные из CSV, пропуская строки с пропущенными значениями."""
        self.data = pd.read_csv(self.input_csv_path)
        # Удаляем строки, где хотя бы одна из колонок (longitude или latitude) NaN
        self.data = self.data.dropna(subset=["longitude", "latitude"])

    def get_point_by_index(self, index):
        """
        Возвращает координаты точки по её индексу.

        :param index: Идентификатор точки.
        :return: Кортеж (longitude, latitude) или None, если точка не найдена.
        """
        point = self.data[self.data["index"] == index]
        if point.empty:
            return None
        return (point["longitude"].values[0], point["latitude"].values[0])

    def find_points_in_radius(self, center_index, radius_m):
        """
        Находит все точки в заданном радиусе (в километрах) от указанной точки.

        :param center_index: Индекс центральной точки.
        :param radius_km: Радиус поиска в километрах.
        :return: DataFrame с точками в радиусе или None, если центральная точка не найдена.
        """
        center_coords = self.get_point_by_index(center_index)
        if center_coords is None:
            print(f"Точка с индексом {center_index} не найдена.")
            return None

        # Преобразуем данные в массив NumPy для быстрых вычислений
        points = self.data[["longitude", "latitude"]].values
        indices = self.data["index"].values

        # Вычисляем расстояния от центра до всех точек
        distances_m = np.array(
            [geodesic(center_coords, (lon, lat)).meters for lon, lat in points]
        )

        # Находим точки внутри радиуса
        mask = distances_m <= radius_m
        result = self.data.iloc[mask].copy()
        result["distance_m"] = distances_m[mask]

        return result

    def save_to_csv(self, df, output_csv_path):
        """
        Сохраняет DataFrame в CSV-файл.

        :param df: DataFrame для сохранения.
        :param output_csv_path: Путь к выходному файлу.
        """
        if df is not None:
            df.to_csv(output_csv_path, index=False)
            print(f"Результат сохранён в {output_csv_path}")
        else:
            print("Нет данных для сохранения.")


if __name__ == "__main__":
    cli = CLI()
    args = cli.parse_args()

    processor = CSVGeoProcessor(args.input)
    points_in_radius = processor.find_points_in_radius(
        center_index=args.center, radius_m=args.radius
    )
    processor.save_to_csv(points_in_radius, args.output)
