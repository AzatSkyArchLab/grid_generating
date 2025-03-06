from flask import Flask, render_template, request, jsonify
from shapely.geometry import Polygon, mapping, shape
import json
import math
import os
from grid.grid_shapely import create_optimal_grid

app = Flask(__name__)


@app.route('/')
def index():
    """Рендерит главную страницу"""
    return render_template('index.html')


@app.route('/generate-grid', methods=['POST'])
def generate_grid():
    """Принимает полигон из фронтенда и генерирует сетку"""
    data = request.json

    print("Получены данные:", data)

    # Извлекаем координаты полигона из GeoJSON
    polygon_coords = data['polygon']['coordinates'][0]
    print("Координаты полигона:", polygon_coords)

    # Создаем полигон Shapely из координат
    polygon = Polygon(polygon_coords)

    # Получаем параметры сетки в метрах
    cell_width = float(data.get('cellWidth', 10))
    cell_height = float(data.get('cellHeight', 10))

    # Получаем настройки проекции
    use_projection = data.get('useProjection', True)
    projection_type = data.get('projectionType', 'auto')

    print(f"Параметры сетки: ширина={cell_width}м, высота={cell_height}м")
    print(f"Использование проекции: {use_projection}, тип: {projection_type}")

    try:
        # Генерируем сетку с использованием проекции для правильной формы ячеек
        print("Вызываем create_optimal_grid...")
        cells, cell_info = create_optimal_grid(
            polygon,
            cell_width=cell_width,
            cell_height=cell_height,
            use_projection=use_projection,
            projection_type=projection_type
        )
        print(f"Сгенерировано {len(cells)} ячеек")

        # Преобразуем ячейки в GeoJSON для отправки обратно на фронтенд
        cells_geojson = {
            "type": "FeatureCollection",
            "features": []
        }

        for cell in cells:
            cell_type = cell_info.get(cell, {}).get('type', 'unknown')
            feature = {
                "type": "Feature",
                "properties": {
                    "type": cell_type
                },
                "geometry": mapping(cell)
            }
            cells_geojson["features"].append(feature)

        # Подсчитываем количество ячеек каждого типа
        full_cells_count = sum(1 for cell in cells if cell_info.get(cell, {}).get('type') == 'full')
        trimmed_cells_count = sum(1 for cell in cells if cell_info.get(cell, {}).get('type') == 'trimmed')

        return jsonify({
            "status": "success",
            "grid": cells_geojson,
            "info": {
                "fullCellsCount": full_cells_count,
                "trimmedCellsCount": trimmed_cells_count,
                "totalCellsCount": len(cells),
                "cellWidth": cell_width,
                "cellHeight": cell_height
            }
        })

    except Exception as e:
        import traceback
        print("Ошибка при генерации сетки:")
        print(traceback.format_exc())
        return jsonify({"status": "error", "message": str(e)})


if __name__ == '__main__':
    app.run(debug=True)