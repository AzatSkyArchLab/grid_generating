import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString, Polygon, MultiPolygon
from shapely.ops import transform, unary_union
from shapely.affinity import rotate, translate
import math
import pyproj
from functools import partial


class Grid:
    def __init__(self, boundary_curve, cell_width=141, cell_height=180, tolerance=0.001,
                 use_projection=True, projection_type='auto'):
        """
        Инициализация объекта Grid с границей и размерами ячеек

        Аргументы:
            boundary_curve: Входная граничная кривая (замкнутый полигон Shapely)
            cell_width: Ширина каждой ячейки сетки в метрах
            cell_height: Высота каждой ячейки сетки в метрах
            tolerance: Допуск для геометрических операций (по умолчанию 0.001)
            use_projection: Использовать ли проекцию для географических координат
            projection_type: 'auto', 'utm', 'web_mercator' или 'lcc'
        """
        # Проверка валидности входных параметров
        if cell_width <= 0 or cell_height <= 0:
            raise ValueError("Размеры ячейки должны быть положительными числами")

        if tolerance <= 0:
            raise ValueError("Геометрический допуск должен быть положительным числом")

        # Проверка, что граница является полигоном и замкнута
        if not isinstance(boundary_curve, Polygon):
            raise ValueError("Граница должна быть полигоном Shapely")

        # Сохраняем входные параметры
        self.original_boundary = boundary_curve
        self.cell_width = float(cell_width)
        self.cell_height = float(cell_height)
        self.tolerance = tolerance
        self.use_projection = use_projection
        self.projection_type = projection_type

        # Параметры для отслеживания оптимальной конфигурации
        self.cells = []
        self.angle = 0
        self.offset_x = 0
        self.offset_y = 0

        # Настраиваем проекцию, если необходимо
        if self.use_projection:
            self.setup_projection()
            # Преобразуем границу в проекцию
            self.boundary = self.project_geometry(self.original_boundary)
            print(f"Граница преобразована в проекцию: {self.proj_name}")
        else:
            self.boundary = self.original_boundary
            print("Используются исходные координаты без проекции")

    def setup_projection(self):
        """
        Настраивает подходящую проекцию на основе центра границы
        """
        # Получаем центр границы
        centroid = self.original_boundary.centroid
        center_lon, center_lat = centroid.x, centroid.y

        # Проверяем, выглядят ли координаты как географические
        is_geographic = (-180 <= center_lon <= 180) and (-90 <= center_lat <= 90)

        if not is_geographic:
            print("Координаты не выглядят как географические, проекция не требуется")
            self.use_projection = False
            return

        # Определяем, какую проекцию использовать
        if self.projection_type == 'auto' or self.projection_type == 'utm':
            # Рассчитываем номер зоны UTM
            utm_zone = int((center_lon + 180) / 6) + 1
            hemisphere = 'north' if center_lat >= 0 else 'south'

            # Создаем проекцию UTM
            self.proj_from = pyproj.CRS('EPSG:4326')  # WGS84
            if hemisphere == 'north':
                self.proj_to = pyproj.CRS(f"+proj=utm +zone={utm_zone} +datum=WGS84 +units=m +no_defs")
            else:
                self.proj_to = pyproj.CRS(f"+proj=utm +zone={utm_zone} +south +datum=WGS84 +units=m +no_defs")

            self.proj_name = f"UTM Zone {utm_zone} {hemisphere.upper()}"

        elif self.projection_type == 'web_mercator':
            # Web Mercator (используется в Google Maps, OpenStreetMap и т.д.)
            self.proj_from = pyproj.CRS('EPSG:4326')  # WGS84
            self.proj_to = pyproj.CRS('EPSG:3857')  # Web Mercator

            self.proj_name = "Web Mercator (EPSG:3857)"

        elif self.projection_type == 'lcc':
            # Lambert Conformal Conic (хорошо для средних широт)
            self.proj_from = pyproj.CRS('EPSG:4326')  # WGS84
            self.proj_to = pyproj.CRS(f"+proj=lcc +lat_1={center_lat - 5} +lat_2={center_lat + 5} "
                                      f"+lat_0={center_lat} +lon_0={center_lon} "
                                      f"+x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs")

            self.proj_name = f"Lambert Conformal Conic (centered at {center_lat:.2f}, {center_lon:.2f})"

        else:
            # По умолчанию используем Web Mercator
            self.proj_from = pyproj.CRS('EPSG:4326')  # WGS84
            self.proj_to = pyproj.CRS('EPSG:3857')  # Web Mercator

            self.proj_name = "Default Web Mercator (EPSG:3857)"

        # Создаем трансформеры
        self.project = pyproj.Transformer.from_crs(self.proj_from, self.proj_to, always_xy=True).transform
        self.unproject = pyproj.Transformer.from_crs(self.proj_to, self.proj_from, always_xy=True).transform

    def project_geometry(self, geometry):
        """
        Преобразует геометрию из исходной системы координат в проекцию
        """
        if not self.use_projection:
            return geometry

        return transform(self.project, geometry)

    def unproject_geometry(self, geometry):
        """
        Преобразует геометрию из проекции обратно в исходную систему координат
        """
        if not self.use_projection:
            return geometry

        return transform(self.unproject, geometry)

    def extract_segment_directions(self):
        """
        Извлечение доминирующих направлений из сегментов границы
        и расчет оптимального направления сетки на основе минимального габаритного прямоугольника
        """
        # Получаем направление минимального габаритного прямоугольника
        min_bbox_angle = self._get_min_bounding_box_angle()

        # Получаем координаты границы
        coords = list(self.boundary.exterior.coords)

        # Рассчитываем длину и направление каждого сегмента
        segments = []
        total_length = 0

        for i in range(len(coords) - 1):
            start_pt = coords[i]
            end_pt = coords[i + 1]

            dx = end_pt[0] - start_pt[0]
            dy = end_pt[1] - start_pt[1]
            length = math.sqrt(dx * dx + dy * dy)
            total_length += length

            # Пропускаем очень короткие сегменты
            if length < self.tolerance * 100:
                continue

            # Нормализуем направление к [0, π)
            angle = math.atan2(dy, dx) % math.pi

            segments.append({
                'angle': angle,
                'length': length,
                'start': start_pt,
                'end': end_pt
            })

        # Если нет достаточно длинных сегментов, используем стандартные направления и МГП
        if not segments:
            print("Предупреждение: не найдено значимых сегментов, используем стандартные направления и МГП")
            all_directions = [0, math.pi / 4, math.pi / 2, 3 * math.pi / 4]
            if min_bbox_angle is not None:
                all_directions.append(min_bbox_angle)
                print(f"Добавлено направление МГП: {round(math.degrees(min_bbox_angle), 1)}°")
            return sorted(list(set(all_directions)))

        # Группируем по направлениям (с точностью до 3 градусов)
        angle_tolerance = 0.052  # ~3 градуса
        direction_groups = {}

        # Если у нас есть угол МГП, добавляем его как отдельную группу с высокой значимостью
        if min_bbox_angle is not None:
            # Проверяем, есть ли уже похожее направление в сегментах
            bbox_group_exists = False
            for segment in segments:
                if abs(segment['angle'] - min_bbox_angle) < angle_tolerance or abs(
                        segment['angle'] - min_bbox_angle - math.pi) < angle_tolerance:
                    bbox_group_exists = True
                    break

            # Если нет похожего направления, добавляем его как виртуальный сегмент
            if not bbox_group_exists:
                # Создаем виртуальный сегмент для направления МГП с высокой значимостью
                direction_groups[min_bbox_angle] = {
                    'segments': [],
                    'total_length': total_length * 0.5,  # Придаем высокую значимость (50% от общей длины)
                    'is_bbox': True  # Помечаем, что это направление МГП
                }

        # Группируем сегменты по направлениям
        for segment in segments:
            angle = segment['angle']
            found_group = False

            for group_angle in direction_groups.keys():
                if abs(angle - group_angle) < angle_tolerance or abs(angle - group_angle - math.pi) < angle_tolerance:
                    direction_groups[group_angle]['segments'].append(segment)
                    direction_groups[group_angle]['total_length'] += segment['length']
                    found_group = True
                    break

            if not found_group:
                direction_groups[angle] = {
                    'segments': [segment],
                    'total_length': segment['length'],
                    'is_bbox': False
                }

        # Вычисляем дополнительные метрики значимости для каждой группы направлений
        for angle, group in direction_groups.items():
            # Если это группа МГП без сегментов, пропускаем некоторые вычисления
            if group.get('is_bbox', False) and not group['segments']:
                group['count'] = 0
                group['avg_length'] = 0
                group['percentage'] = (group['total_length'] / total_length) * 100
                # Очень высокая значимость для МГП
                group['significance'] = group['percentage'] * 3
                continue

            # Процент от общей длины всех сегментов
            group['percentage'] = (group['total_length'] / total_length) * 100

            # Количество сегментов в группе
            group['count'] = len(group['segments'])

            # Средняя длина сегмента в группе
            group['avg_length'] = group['total_length'] / group['count'] if group['count'] > 0 else 0

            # Комплексная оценка значимости направления (с большим весом процента длины)
            # Даем дополнительный бонус, если это направление МГП
            base_significance = (group['percentage'] * 2) + (group['count'] * 0.5)
            group['significance'] = base_significance * 1.5 if group.get('is_bbox', False) else base_significance

        # Сортируем направления по значимости
        sorted_directions = sorted(direction_groups.keys(),
                                   key=lambda a: direction_groups[a]['significance'],
                                   reverse=True)

        # Выбираем основные направления на основе их значимости
        # Отбираем направления с значимостью выше среднего или топ-3, что больше
        significance_values = [direction_groups[a]['significance'] for a in sorted_directions]
        avg_significance = sum(significance_values) / len(significance_values) if significance_values else 0

        main_directions = []
        for angle in sorted_directions:
            if direction_groups[angle]['significance'] > avg_significance or len(main_directions) < 3:
                main_directions.append(angle)

            # Ограничиваем количество основных направлений
            if len(main_directions) >= 5:
                break

        # Создаем список всех тестируемых направлений
        all_directions = []

        # Добавляем основные направления и их перпендикуляры
        for direction in main_directions:
            all_directions.append(direction)

            # Добавляем перпендикулярное направление к каждому основному
            perpendicular = (direction + math.pi / 2) % math.pi
            all_directions.append(perpendicular)

        # Также добавляем стандартные направления для полноты
        standard_directions = [0, math.pi / 4, math.pi / 2, 3 * math.pi / 4]
        for direction in standard_directions:
            if all(abs(direction - d) > angle_tolerance for d in all_directions):
                all_directions.append(direction)

        # Убираем дубликаты и сортируем направления
        all_directions = list(set(all_directions))
        all_directions.sort()

        # Выводим информацию о найденных направлениях
        print(f"Основные направления сегментов (градусы):")
        for direction in main_directions:
            percentage = direction_groups[direction]['percentage']
            segment_count = direction_groups[direction]['count']
            significance = direction_groups[direction]['significance']
            is_bbox = direction_groups[direction].get('is_bbox', False)
            bbox_info = " (МГП)" if is_bbox else ""
            print(f"  - {round(math.degrees(direction), 1)}°{bbox_info} (значимость: {significance:.1f}, "
                  f"доля длины: {percentage:.1f}%, сегментов: {segment_count})")

        print(f"Все тестируемые направления (градусы): {[round(math.degrees(d), 1) for d in all_directions]}")

        # Сохраняем информацию о группах направлений для дальнейшего использования
        self.direction_groups = direction_groups

        return all_directions

    def _get_min_bounding_box_angle(self):
        """
        Определяет угол минимального габаритного прямоугольника (МГП)

        Возвращает:
            угол в радианах от 0 до π, или None в случае ошибки
        """
        try:
            from shapely.geometry import box
            from shapely.affinity import rotate, translate

            # Получаем координаты полигона
            coords = list(self.boundary.exterior.coords)

            if len(coords) < 4:
                print("Недостаточно координат для расчета МГП")
                return None

            # Начальные значения для поиска МГП
            min_area = float('inf')
            optimal_angle = 0

            # Ищем угол с минимальной площадью ограничивающего прямоугольника
            # Проверяем углы с шагом 1 градус для скорости
            for angle_deg in range(0, 180, 1):
                # Преобразуем в радианы
                angle_rad = math.radians(angle_deg)

                # Поворачиваем полигон
                rotated = rotate(self.boundary, -angle_deg, origin='centroid')

                # Получаем ограничивающий прямоугольник
                minx, miny, maxx, maxy = rotated.bounds

                # Вычисляем площадь
                area = (maxx - minx) * (maxy - miny)

                # Если нашли меньшую площадь, запоминаем угол
                if area < min_area:
                    min_area = area
                    optimal_angle = angle_rad

            # Точная подстройка угла с шагом 0.1 градус вокруг найденного оптимального
            fine_step = math.radians(0.1)
            for fine_angle in [optimal_angle + i * fine_step for i in range(-10, 11)]:
                angle_deg = math.degrees(fine_angle)

                # Поворачиваем полигон
                rotated = rotate(self.boundary, -angle_deg, origin='centroid')

                # Получаем ограничивающий прямоугольник
                minx, miny, maxx, maxy = rotated.bounds

                # Вычисляем площадь
                area = (maxx - minx) * (maxy - miny)

                # Если нашли меньшую площадь, запоминаем угол
                if area < min_area:
                    min_area = area
                    optimal_angle = fine_angle

            # Нормализуем угол к [0, π)
            optimal_angle = optimal_angle % math.pi

            print(f"Найден угол МГП: {round(math.degrees(optimal_angle), 1)}°")

            return optimal_angle

        except Exception as e:
            print(f"Ошибка при определении угла МГП: {e}")
            return None

    def generate_grid_with_exact_size(self, angle, offset_x, offset_y):
        """
        Генерация сетки с заданной ориентацией и смещением
        с сохранением точного размера ячеек и расширением в отрицательном направлении
        """
        # Получаем ограничивающий прямоугольник границы
        minx, miny, maxx, maxy = self.boundary.bounds

        # Вычисляем центр границы для поворота
        center_x = (minx + maxx) / 2
        center_y = (miny + maxy) / 2

        # Поворачиваем границу для выравнивания с осями координат
        rotated_boundary = rotate(self.boundary, -math.degrees(angle), origin=(center_x, center_y), use_radians=False)

        # Получаем ограничивающий прямоугольник повернутой границы
        rot_minx, rot_miny, rot_maxx, rot_maxy = rotated_boundary.bounds

        # ИЗМЕНЕНО: Расширяем сетку на одну ячейку в отрицательном направлении
        # для обеспечения полного покрытия
        grid_x = math.floor(rot_minx / self.cell_width) * self.cell_width - self.cell_width + offset_x
        grid_y = math.floor(rot_miny / self.cell_height) * self.cell_height - self.cell_height + offset_y

        # Вычисляем необходимое количество ячеек с запасом в обе стороны
        cols = math.ceil((rot_maxx - grid_x) / self.cell_width) + 2  # +2 вместо +1
        rows = math.ceil((rot_maxy - grid_y) / self.cell_height) + 2  # +2 вместо +1

        # Создаем словарь для хранения информации о положении ячеек
        cell_grid = {}  # Словарь (row, col) -> объект ячейки

        # Генерируем ячейки сетки
        cells = []
        for row in range(int(rows)):
            for col in range(int(cols)):
                # Создаем прямоугольник ячейки в повернутой системе координат
                x = grid_x + col * self.cell_width
                y = grid_y + row * self.cell_height

                # Создаем углы прямоугольника
                rect_pts = [
                    (x, y),
                    (x + self.cell_width, y),
                    (x + self.cell_width, y + self.cell_height),
                    (x, y + self.cell_height)
                ]

                # Создаем полигон ячейки
                cell = Polygon(rect_pts)

                # Поворачиваем обратно в исходное пространство
                cell = rotate(cell, math.degrees(angle), origin=(center_x, center_y), use_radians=False)

                cells.append(cell)

                # Сохраняем информацию о положении ячейки
                cell_grid[(row, col)] = {
                    'cell': cell,
                    'row': row,
                    'col': col
                }

        return cells, cell_grid

    def is_cell_inside(self, cell):
        """
        Проверка, полностью ли ячейка находится внутри границы
        с улучшенной точностью
        """
        try:
            # Уменьшаем размер буфера для более точной классификации
            reduced_tolerance = self.tolerance * 0.1
            return self.boundary.contains(cell.buffer(-reduced_tolerance))
        except:
            return False

    def is_cell_intersecting(self, cell):
        """
        Улучшенная проверка, пересекается ли ячейка с границей
        """
        try:
            # Более надежная проверка пересечения
            intersects = self.boundary.intersects(cell)
            contains = self.is_cell_inside(cell)

            # Ячейка пересекается, если она пересекает границу, но не полностью внутри
            return intersects and not contains
        except:
            return False

    def trim_cell_by_boundary(self, cell):
        """
        Обрезать ячейку по границе, сохраняя только часть внутри
        """
        try:
            # Пересекаем ячейку с границей
            trimmed = cell.intersection(self.boundary)

            # Проверяем, что результат не пустой
            # Убрана проверка минимального размера, чтобы сохранить все ячейки на границе
            if trimmed.is_empty:
                return None

            return trimmed

        except Exception as e:
            print(f"Ошибка обрезки: {str(e)}")
            return None

    def calculate_grid_quality(self, cells, cell_info):
        """
        Оценивает качество сетки с акцентом на количество полных ячеек
        и размеры фрагментов
        """
        # Подсчитываем количество ячеек разных типов
        full_cells = [c for c in cells if cell_info.get(c, {}).get('type') == 'full']
        trimmed_cells = [c for c in cells if cell_info.get(c, {}).get('type') == 'trimmed']

        # Целевая площадь полной ячейки
        target_area = self.cell_width * self.cell_height

        # Оцениваем качество обрезанных ячеек по их размеру
        trimmed_quality = 0
        for cell in trimmed_cells:
            # Отношение площади фрагмента к площади полной ячейки
            relative_size = cell.area / target_area

            # Изменяем оценку качества для маленьких фрагментов:
            # не штрафуем слишком сильно, но и не поощряем слишком маленькие фрагменты
            if relative_size < 0.05:
                trimmed_quality -= 0.1  # Минимальный штраф за очень маленькие фрагменты
            elif relative_size > 0.75:
                trimmed_quality += 0.5  # Бонус за крупные фрагменты

        # Основной критерий - количество полных ячеек
        full_cells_score = len(full_cells) * 10

        # Дополнительный критерий - качество обрезанных ячеек
        # с меньшим весом, чем полные ячейки
        quality = full_cells_score + trimmed_quality

        return {
            'quality': quality,
            'full_cells': len(full_cells),
            'trimmed_cells': len(trimmed_cells),
            'quality_details': {
                'full_cells_score': full_cells_score,
                'trimmed_quality': trimmed_quality
            }
        }

    def get_alignment_offsets(self, angle):
        """
        Получает набор смещений для тестирования,
        выровненных с сегментами границы
        """
        # Базовые смещения - стандартные доли ячейки
        base_offsets = [
            (0, 0),  # Без смещения
            (self.cell_width / 4, 0),  # 1/4 ширины
            (0, self.cell_height / 4),  # 1/4 высоты
            (self.cell_width / 4, self.cell_height / 4),  # 1/4 по обоим направлениям
            (self.cell_width / 2, 0),  # 1/2 ширины
            (0, self.cell_height / 2),  # 1/2 высоты
            (self.cell_width / 2, self.cell_height / 2)  # 1/2 по обоим направлениям
        ]

        # Дополнительные смещения для выравнивания с сегментами
        # Поворачиваем границу
        center_x, center_y = self.boundary.centroid.x, self.boundary.centroid.y
        rotated_boundary = rotate(self.boundary, -math.degrees(angle), origin=(center_x, center_y), use_radians=False)

        # Получаем координаты повернутой границы
        rot_coords = list(rotated_boundary.exterior.coords)

        # Находим координаты вершин в повернутой системе координат
        vertex_offsets = []
        for x, y in rot_coords:
            # Находим остаток от деления на размер ячейки
            offset_x = x % self.cell_width
            offset_y = y % self.cell_height

            # Если точка близка к краю ячейки, нормализуем смещение
            if offset_x > self.cell_width * 0.9:
                offset_x = 0
            if offset_y > self.cell_height * 0.9:
                offset_y = 0

            vertex_offsets.append((offset_x, offset_y))

        # Убираем дубликаты и объединяем с базовыми смещениями
        all_offsets = base_offsets.copy()
        for offset in vertex_offsets:
            if offset not in all_offsets:
                all_offsets.append(offset)

        return all_offsets[:20]  # Ограничиваем количество смещений для эффективности

    def optimize(self):
        """
        Оптимизация с учетом доминирующих направлений, МГП и весов значимости
        """
        # Получаем основные направления из сегментов границы
        all_directions = self.extract_segment_directions()

        # Рассчитываем значимость для каждого направления
        significance_weights = {}

        # Если у нас есть информация о группах направлений от extract_segment_directions
        if hasattr(self, 'direction_groups') and self.direction_groups:
            # Нормализуем значимость
            max_significance = max(
                [group['significance'] for group in self.direction_groups.values()]) if self.direction_groups else 1.0

            for angle, group in self.direction_groups.items():
                # Если это направление МГП, даем ему максимальный вес
                if group.get('is_bbox', False):
                    significance_weights[angle] = 3.0
                    print(f"Направлению МГП {round(math.degrees(angle), 1)}° присвоен максимальный вес 3.0")
                else:
                    # Нормализованная значимость от 1.0 до 3.0
                    normalized_significance = 1.0 + (group['significance'] / max_significance) * 2.0
                    significance_weights[angle] = normalized_significance

                # Также добавляем вес для перпендикулярного направления с меньшим весом
                perpendicular = (angle + math.pi / 2) % math.pi
                # Для МГП перпендикуляр тоже важен
                if group.get('is_bbox', False):
                    significance_weights[perpendicular] = 2.5
                else:
                    significance_weights[perpendicular] = 1.0 + (group['significance'] / max_significance)
        else:
            # Если информация о группах недоступна, используем равные веса
            for angle in all_directions:
                significance_weights[angle] = 1.0

        # Настраиваем отслеживание лучшей сетки
        best_quality = -float('inf')
        optimal_angle = 0
        optimal_offset_x = 0
        optimal_offset_y = 0
        optimal_cells = None
        optimal_cell_info = None

        # Отслеживаем лучшие конфигурации для второй фазы
        top_configs = []

        # ФАЗА 1: Широкий поиск с приоритетом доминирующих направлений
        for angle in all_directions:
            # Получаем вес значимости для текущего направления (или 1.0 по умолчанию)
            significance = significance_weights.get(angle, 1.0)

            # Вывод информации с указанием значимости направления
            print(f"Тестируем угол {math.degrees(angle):.1f}° (значимость: {significance:.2f})")

            # Получаем набор смещений для тестирования
            offsets = self.get_alignment_offsets(angle)

            # Основной код тестирования вариантов размещения сетки
            for offset_x, offset_y in offsets:
                # Генерируем тестовую сетку
                all_cells, cell_grid = self.generate_grid_with_exact_size(angle, offset_x, offset_y)

                # Анализируем и классифицируем ячейки
                cell_info = {}
                valid_cells = []

                # Сначала находим все внутренние ячейки
                for cell_id, cell_data in cell_grid.items():
                    cell = cell_data['cell']
                    row, col = cell_data['row'], cell_data['col']

                    if self.is_cell_inside(cell):
                        cell_info[cell] = {
                            'row': row,
                            'col': col,
                            'type': 'full'
                        }
                        valid_cells.append(cell)

                # Затем находим все пересекающиеся с границей ячейки
                for cell_id, cell_data in cell_grid.items():
                    cell = cell_data['cell']
                    row, col = cell_data['row'], cell_data['col']

                    # Если ячейка еще не классифицирована как внутренняя
                    if cell not in cell_info:
                        if self.is_cell_intersecting(cell):
                            # Для частичных ячеек обрезаем их по границе
                            trimmed_cell = self.trim_cell_by_boundary(cell)
                            if trimmed_cell:
                                cell_info[trimmed_cell] = {
                                    'row': row,
                                    'col': col,
                                    'type': 'trimmed'
                                }
                                valid_cells.append(trimmed_cell)

                # Проверяем наличие ячеек на всех краях границы
                if valid_cells:
                    # Получаем все позиции ячеек
                    cell_positions = {}
                    for c, info in cell_info.items():
                        cell_positions[(info['row'], info['col'])] = True

                    # Получаем мин/макс координаты для обработанных ячеек
                    rows = [info['row'] for c, info in cell_info.items()]
                    cols = [info['col'] for c, info in cell_info.items()]

                    if rows and cols:
                        min_row, max_row = min(rows), max(rows)
                        min_col, max_col = min(cols), max(cols)

                        # Дополнительно проверяем ячейки на краях
                        for row in range(min_row - 1, max_row + 2):
                            for col in range(min_col - 1, max_col + 2):
                                # Проверяем только ячейки по периметру и не обработанные ранее
                                is_perimeter = (row == min_row - 1 or row == max_row + 1 or
                                                col == min_col - 1 or col == max_col + 1)

                                if is_perimeter and (row, col) in cell_grid:
                                    test_cell = cell_grid[(row, col)]['cell']

                                    # Если эта ячейка еще не обработана
                                    if test_cell not in cell_info and not any(c.equals(test_cell) for c in cell_info):
                                        # Проверяем пересечение с границей
                                        if self.boundary.intersects(test_cell):
                                            trimmed_cell = self.trim_cell_by_boundary(test_cell)
                                            if trimmed_cell:
                                                cell_info[trimmed_cell] = {
                                                    'row': row,
                                                    'col': col,
                                                    'type': 'trimmed'
                                                }
                                                valid_cells.append(trimmed_cell)

                # Если нет действительных ячеек, пропускаем конфигурацию
                if not valid_cells:
                    continue

                # Рассчитываем качество этой сетки
                grid_quality = self.calculate_grid_quality(valid_cells, cell_info)

                # НОВОЕ: Учитываем значимость направления в оценке качества
                # Корректируем качество на основе значимости направления
                adjusted_quality = grid_quality['quality'] * significance

                # Сохраняем информацию для топ-конфигураций
                full_cells_count = len([c for c in valid_cells if cell_info.get(c, {}).get('type') == 'full'])

                config_data = {
                    'angle': angle,
                    'offset_x': offset_x,
                    'offset_y': offset_y,
                    'quality': grid_quality['quality'],
                    'adjusted_quality': adjusted_quality,
                    'full_cells': full_cells_count,
                    'trimmed_cells': len(valid_cells) - full_cells_count,
                    'significance': significance
                }

                # Добавляем в топ-конфигурации, если это одно из лучших решений
                if len(top_configs) < 5 or config_data['adjusted_quality'] > \
                        min(top_configs, key=lambda x: x['adjusted_quality'])['adjusted_quality']:
                    top_configs.append(config_data)
                    # Сортируем и оставляем только топ-5
                    top_configs.sort(key=lambda x: x['adjusted_quality'], reverse=True)
                    if len(top_configs) > 5:
                        top_configs.pop()

                # Обновляем лучшую конфигурацию на основе скорректированного качества
                if adjusted_quality > best_quality:
                    best_quality = adjusted_quality
                    optimal_angle = angle
                    optimal_offset_x = offset_x
                    optimal_offset_y = offset_y
                    optimal_cells = valid_cells
                    optimal_cell_info = cell_info

                    print(f"Новая лучшая конфигурация: угол={math.degrees(optimal_angle):.1f}°, "
                          f"смещение=({optimal_offset_x:.2f}, {optimal_offset_y:.2f}), "
                          f"полных ячеек: {full_cells_count}, "
                          f"обрезанных: {len(valid_cells) - full_cells_count}, "
                          f"значимость направления: {significance:.2f}")

        # ФАЗА 2: Уточнение лучших конфигураций
        print("Уточняем лучшие конфигурации...")

        # Проверяем все топ-конфигурации с небольшими вариациями смещений
        for config in top_configs:
            angle = config['angle']
            base_offset_x = config['offset_x']
            base_offset_y = config['offset_y']
            significance = config['significance']

            # Создаем небольшие вариации смещений вокруг базового
            variation_step = min(self.cell_width, self.cell_height) / 10
            variations = [
                (0, 0),  # Базовое смещение
                (variation_step, 0),
                (-variation_step, 0),
                (0, variation_step),
                (0, -variation_step),
                (variation_step, variation_step),
                (-variation_step, -variation_step),
                (variation_step, -variation_step),
                (-variation_step, variation_step)
            ]

            for dx, dy in variations:
                offset_x = base_offset_x + dx
                offset_y = base_offset_y + dy

                # Нормализуем смещения к диапазону размера ячейки
                offset_x = offset_x % self.cell_width
                offset_y = offset_y % self.cell_height

                # Генерируем тестовую сетку с точным сохранением размеров ячеек
                all_cells, cell_grid = self.generate_grid_with_exact_size(angle, offset_x, offset_y)

                # Анализируем и классифицируем ячейки
                cell_info = {}
                valid_cells = []

                # Находим внутренние и граничные ячейки
                for cell_id, cell_data in cell_grid.items():
                    cell = cell_data['cell']
                    row, col = cell_data['row'], cell_data['col']

                    if self.is_cell_inside(cell):
                        cell_info[cell] = {
                            'row': row,
                            'col': col,
                            'type': 'full'
                        }
                        valid_cells.append(cell)
                    elif self.is_cell_intersecting(cell):
                        # Для частичных ячеек обрезаем их по границе
                        trimmed_cell = self.trim_cell_by_boundary(cell)
                        if trimmed_cell:
                            cell_info[trimmed_cell] = {
                                'row': row,
                                'col': col,
                                'type': 'trimmed'
                            }
                            valid_cells.append(trimmed_cell)

                # Проверяем ячейки на краях с той же логикой, что и в основной фазе
                if valid_cells:
                    # Получаем все позиции ячеек
                    cell_positions = {}
                    for c, info in cell_info.items():
                        cell_positions[(info['row'], info['col'])] = True

                    # Получаем мин/макс координаты для обработанных ячеек
                    rows = [info['row'] for c, info in cell_info.items()]
                    cols = [info['col'] for c, info in cell_info.items()]

                    if rows and cols:
                        min_row, max_row = min(rows), max(rows)
                        min_col, max_col = min(cols), max(cols)

                        # Дополнительно проверяем ячейки на краях
                        for row in range(min_row - 1, max_row + 2):
                            for col in range(min_col - 1, max_col + 2):
                                is_perimeter = (row == min_row - 1 or row == max_row + 1 or
                                                col == min_col - 1 or col == max_col + 1)

                                if is_perimeter and (row, col) in cell_grid:
                                    test_cell = cell_grid[(row, col)]['cell']

                                    if test_cell not in cell_info and not any(c.equals(test_cell) for c in cell_info):
                                        if self.boundary.intersects(test_cell):
                                            trimmed_cell = self.trim_cell_by_boundary(test_cell)
                                            if trimmed_cell:
                                                cell_info[trimmed_cell] = {
                                                    'row': row,
                                                    'col': col,
                                                    'type': 'trimmed'
                                                }
                                                valid_cells.append(trimmed_cell)

                # Если нет действительных ячеек, пропускаем конфигурацию
                if not valid_cells:
                    continue

                # Рассчитываем качество этой сетки
                grid_quality = self.calculate_grid_quality(valid_cells, cell_info)

                # Учитываем значимость направления
                adjusted_quality = grid_quality['quality'] * significance

                # Обновляем, если эта конфигурация лучше текущей лучшей
                if adjusted_quality > best_quality:
                    best_quality = adjusted_quality
                    optimal_angle = angle
                    optimal_offset_x = offset_x
                    optimal_offset_y = offset_y
                    optimal_cells = valid_cells
                    optimal_cell_info = cell_info

                    full_cells_count = len([c for c in valid_cells if cell_info.get(c, {}).get('type') == 'full'])
                    print(f"Уточнение: новая лучшая конфигурация: угол={math.degrees(optimal_angle):.1f}°, "
                          f"смещение=({optimal_offset_x:.2f}, {optimal_offset_y:.2f}), "
                          f"полных ячеек: {full_cells_count}, "
                          f"обрезанных: {len(valid_cells) - full_cells_count}")

        # Если не нашли ни одной конфигурации, генерируем сетку по умолчанию
        if optimal_cells is None:
            print("Не удалось найти оптимальную конфигурацию, используем стандартную")
            all_cells, cell_grid = self.generate_grid_with_exact_size(0, 0, 0)

            # Классифицируем ячейки
            cell_info = {}
            valid_cells = []

            for cell_id, cell_data in cell_grid.items():
                cell = cell_data['cell']
                row, col = cell_data['row'], cell_data['col']

                if self.is_cell_inside(cell):
                    cell_info[cell] = {
                        'row': row,
                        'col': col,
                        'type': 'full'
                    }
                    valid_cells.append(cell)
                elif self.is_cell_intersecting(cell):
                    # Для частичных ячеек обрезаем их по границе
                    trimmed_cell = self.trim_cell_by_boundary(cell)
                    if trimmed_cell:
                        cell_info[trimmed_cell] = {
                            'row': row,
                            'col': col,
                            'type': 'trimmed'
                        }
                        valid_cells.append(trimmed_cell)

            optimal_cells = valid_cells
            optimal_cell_info = cell_info
            optimal_angle = 0
            optimal_offset_x = 0
            optimal_offset_y = 0

        # Запоминаем оптимальную конфигурацию
        self.angle = optimal_angle
        self.offset_x = optimal_offset_x
        self.offset_y = optimal_offset_y

        # Выводим информацию о найденной оптимальной сетке
        print(f"Оптимальная конфигурация: угол={math.degrees(self.angle):.1f}°, "
              f"смещение=({self.offset_x:.2f}, {self.offset_y:.2f})")

        if optimal_cell_info:
            full_cells = [c for c in optimal_cells if optimal_cell_info.get(c, {}).get('type') == 'full']
            trimmed_cells = [c for c in optimal_cells if optimal_cell_info.get(c, {}).get('type') == 'trimmed']
            print(f"Создано {len(full_cells)} полных ячеек и {len(trimmed_cells)} обрезанных ячеек")
            print(f"Всего ячеек: {len(optimal_cells)}")

        # Если используется проекция, преобразуем результаты обратно в исходные координаты
        if self.use_projection:
            # Преобразуем все ячейки обратно в географические координаты
            projected_cells = optimal_cells
            projected_cell_info = optimal_cell_info

            # Преобразуем каждую ячейку и создаем новый словарь с информацией
            unprojected_cells = []
            unprojected_cell_info = {}

            for cell in projected_cells:
                # Преобразуем ячейку обратно в географические координаты
                unprojected_cell = self.unproject_geometry(cell)
                unprojected_cells.append(unprojected_cell)

                # Копируем информацию о ячейке
                if cell in projected_cell_info:
                    unprojected_cell_info[unprojected_cell] = projected_cell_info[cell]

            # Обновляем результаты
            self.cells = unprojected_cells
            return unprojected_cells, unprojected_cell_info
        else:
            # Если проекция не используется, возвращаем результаты как есть
            self.cells = optimal_cells
            return optimal_cells, optimal_cell_info


def create_optimal_grid(input_boundary, cell_width=141, cell_height=180,
                        use_projection=True, projection_type='auto'):
    """
    Создать оптимальную сетку, которая заполняет граничную кривую,
    обрезая ячейки по краям.

    Аргументы:
        input_boundary: Входная граничная кривая (полигон Shapely)
        cell_width: Ширина каждой ячейки сетки в метрах (по умолчанию: 141)
        cell_height: Высота каждой ячейки сетки в метрах (по умолчанию: 180)
        use_projection: Использовать ли проекцию для географических координат
        projection_type: 'auto', 'utm', 'web_mercator' или 'lcc'

    Возвращает:
        Список ячеек сетки (полных и обрезанных) и информацию о них
    """
    try:
        # Создаем объект сетки
        grid = Grid(input_boundary, cell_width, cell_height,
                    use_projection=use_projection, projection_type=projection_type)

        # Находим оптимальную сетку и заполняем границу
        cells, cell_info = grid.optimize()

        return cells, cell_info
    except Exception as e:
        print(f"Ошибка: {str(e)}")
        import traceback
        traceback.print_exc()
        return None, None


def visualize_grid(boundary, cells, cell_info=None):
    """
    Визуализировать границу и сетку

    Аргументы:
        boundary: Граница (полигон Shapely)
        cells: Список ячеек сетки (полигоны Shapely)
        cell_info: Словарь с информацией о ячейках (опционально)
    """
    fig, ax = plt.subplots(figsize=(12, 10))

    # Отображаем границу
    x, y = boundary.exterior.xy
    ax.plot(x, y, color='black', linewidth=2)

    # Если у границы есть внутренние кольца (отверстия), отображаем их
    for interior in boundary.interiors:
        x, y = interior.xy
        ax.plot(x, y, color='black', linewidth=2)

    # Отображаем ячейки
    for cell in cells:
        if cell_info and cell in cell_info:
            cell_type = cell_info[cell]['type']
            if cell_type == 'full':
                color = 'lightblue'
            elif cell_type == 'trimmed':
                color = 'lightgreen'
            else:
                color = 'lightgray'
        else:
            color = 'lightgray'

        if isinstance(cell, Polygon):
            x, y = cell.exterior.xy
            ax.fill(x, y, alpha=0.5, color=color, edgecolor='gray')
        elif isinstance(cell, MultiPolygon):
            for geom in cell.geoms:
                x, y = geom.exterior.xy
                ax.fill(x, y, alpha=0.5, color=color, edgecolor='gray')

    ax.set_aspect('equal')
    ax.set_title('Сетка, заполняющая границу')
    plt.tight_layout()
    plt.show()