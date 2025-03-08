import folium
import pandas as pd
import http.server
import socketserver

# Загрузка данных из CSV
csv_file = "368700.csv"  # Файл должен содержать колонки 'index', 'longitude', 'lattitude'
df = pd.read_csv(csv_file, header=0)
df.columns = df.columns.str.strip()
df = df.dropna(subset=['longitude', 'lattitude'])

# Создание карты (по умолчанию центрируется на первой точке)
map_center = [df['lattitude'].mean(), df['longitude'].mean()]
map_object = folium.Map(location=map_center, zoom_start=12)

# Добавление точек на карту
for _, row in df.iterrows():
    folium.Marker([row['lattitude'], row['longitude']], popup=row['index']).add_to(map_object)

# Сохранение карты в файл
map_file = "map.html"
map_object.save(map_file)

print(f"Карта сохранена в {map_file}. Откройте этот файл в браузере.")

# Запуск локального веб-сервера для просмотра карты
# PORT = 8000
# Handler = http.server.SimpleHTTPRequestHandler

# with socketserver.TCPServer(("", PORT), Handler) as httpd:
#     print(f"Сервер запущен на http://localhost:{PORT}, откройте браузер для просмотра карты.")
#     httpd.serve_forever()