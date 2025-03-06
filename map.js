// Инициализация карты
let map = L.map('map', {
    zoomControl: true, // Добавляем элементы управления зумом
    doubleClickZoom: true // Разрешаем зум двойным щелчком
}).setView([55.69888, 37.63441], 16); // Москва - ЗИЛ АРТ для теста

// Добавление слоя OpenStreetMap
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
}).addTo(map);

// Создание слоя для рисования
let drawnItems = new L.FeatureGroup();
map.addLayer(drawnItems);

// Создание слоя для сетки
let gridLayer = new L.FeatureGroup();
map.addLayer(gridLayer);

// Глобальная переменная для хранения текущего полигона
let currentPolygon = null;

// Настройка контрола для рисования
let drawControl = new L.Control.Draw({
    draw: {
        marker: false,
        circlemarker: false,
        circle: false,
        rectangle: false,
        polyline: false,
        polygon: {
            allowIntersection: false, // Запрещаем самопересечения
            drawError: {
                color: '#e1e100',
                message: '<strong>Ошибка:</strong> самопересечение полигона!'
            },
            shapeOptions: {
                color: '#FF6347' // Томатный цвет для полигона
            }
        }
    },
    edit: {
        featureGroup: drawnItems,
        remove: true,
        poly: {
            allowIntersection: false
        }
    }
});
map.addControl(drawControl);

// Функция для обновления информации о сетке
function updateGridInfo(data) {
    if (!data || !data.info) return;

    const info = data.info;
    const gridInfoDiv = document.getElementById('grid-info');

    // Формируем информацию о сетке для пользователя
    gridInfoDiv.innerHTML = `
        <strong>Размер ячеек:</strong> ${info.cellWidth} × ${info.cellHeight} м<br>
        <strong>Полные ячейки:</strong> ${info.fullCellsCount}<br>
        <strong>Обрезанные ячейки:</strong> ${info.trimmedCellsCount}<br>
        <strong>Всего ячеек:</strong> ${info.totalCellsCount}
    `;
}

// Функция для обновления статуса
function updateStatus(message, isError = false) {
    const statusElement = document.getElementById('status');
    statusElement.textContent = message;
    statusElement.className = isError ? 'status-error' : 'status-success';
}

// Функция для очистки карты
function clearMap() {
    console.log("Очистка карты");
    drawnItems.clearLayers();
    gridLayer.clearLayers();
    currentPolygon = null;
    updateStatus('');
    document.getElementById('grid-info').innerHTML = '';
}

// Функция для генерации сетки на основе полигона
function generateGrid(polygon) {
    if (!polygon) {
        console.log("Нет полигона для генерации сетки");
        return;
    }

    console.log("Генерация сетки для полигона:", polygon);

    // Получаем GeoJSON полигона
    const geojson = polygon.toGeoJSON();

    // Получаем значения размеров ячеек
    const cellWidth = document.getElementById('cell-width').value;
    const cellHeight = document.getElementById('cell-height').value;

    // Получаем настройки проекции
    const projectionType = document.getElementById('projection-type').value;
    const useProjection = projectionType !== 'none';

    console.log(`Размеры ячеек: ${cellWidth} × ${cellHeight} м`);
    console.log(`Проекция: ${projectionType}, использовать: ${useProjection}`);

    // Показываем статус загрузки
    updateStatus('Генерация сетки...');

    // Отправляем запрос на сервер для генерации сетки
    fetch('/generate-grid', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            polygon: geojson.geometry,
            cellWidth: cellWidth,
            cellHeight: cellHeight,
            useProjection: useProjection,
            projectionType: projectionType
        })
    })
    .then(response => {
        console.log("Получен ответ от сервера");
        return response.json();
    })
    .then(data => {
        console.log("Данные от сервера:", data);
        if (data.status === 'success') {
            // Очищаем предыдущую сетку
            gridLayer.clearLayers();

            // Обновляем информацию о сетке
            updateGridInfo(data);

            // Отображаем новую сетку
            L.geoJSON(data.grid, {
                style: function(feature) {
                    const cellType = feature.properties.type;
                    if (cellType === 'full') {
                        return {
                            fillColor: '#6495ED',
                            fillOpacity: 0.6,
                            color: '#4682B4',
                            weight: 1
                        };
                    } else if (cellType === 'trimmed') {
                        return {
                            fillColor: '#90EE90',
                            fillOpacity: 0.6,
                            color: '#3CB371',
                            weight: 1
                        };
                    } else {
                        return {
                            fillColor: '#D3D3D3',
                            fillOpacity: 0.5,
                            color: '#A9A9A9',
                            weight: 1
                        };
                    }
                }
            }).addTo(gridLayer);

            updateStatus(`Сетка успешно сгенерирована! Количество ячеек: ${data.info.totalCellsCount}`);
        } else {
            console.error("Ошибка:", data.message);
            updateStatus(`Ошибка: ${data.message}`, true);
        }
    })
    .catch(error => {
        console.error('Ошибка:', error);
        updateStatus('Произошла ошибка при соединении с сервером', true);
    });
}

// Обработчик события создания полигона
map.on(L.Draw.Event.CREATED, function (event) {
    console.log("Событие CREATED:", event);
    const layer = event.layer;

    // Добавляем слой на карту
    drawnItems.clearLayers(); // Очищаем предыдущие слои
    drawnItems.addLayer(layer);

    // Сохраняем текущий полигон
    currentPolygon = layer;

    // Генерируем сетку
    generateGrid(layer);
});

// Обработчик для удаления последней вершины во время рисования
map.on('draw:drawvertex', function (e) {
    // Получаем текущий полигон, который рисуется
    const drawingPolygon = e.poly;

    // Проверяем, есть ли активная кнопка удаления
    const deleteButtonElement = document.querySelector('.leaflet-draw-actions a[title="Delete last point drawn"]');
    if (deleteButtonElement) {
        // Делаем кнопку активной
        deleteButtonElement.style.backgroundColor = '#f44336';
        deleteButtonElement.style.color = 'white';
    }
});

// Обработчик события редактирования
map.on(L.Draw.Event.EDITED, function (event) {
    console.log("Событие EDITED:", event);
    // Получаем измененные слои
    const layers = event.layers;

    console.log("Измененные слои:", layers);

    layers.eachLayer(function (layer) {
        console.log("Обработка измененного слоя:", layer);
        // Сохраняем текущий полигон
        currentPolygon = layer;

        // Генерируем сетку заново на основе отредактированного полигона
        generateGrid(layer);
    });
});

// Обработчик события удаления
map.on(L.Draw.Event.DELETED, function (event) {
    console.log("Событие DELETED:", event);
    clearMap();
});

// Создание кнопки обновления сетки
const controlDiv = document.createElement('div');
controlDiv.className = 'update-control';
controlDiv.innerHTML = '<button id="update-grid" class="btn">Обновить сетку</button>';

// Добавление кнопки на карту
const controlClass = L.Control.extend({
    options: {
        position: 'topright'
    },
    onAdd: function() {
        return controlDiv;
    }
});
map.addControl(new controlClass());

// Обработчик нажатия на кнопку обновления сетки
document.getElementById('update-grid').addEventListener('click', function() {
    if (currentPolygon) {
        console.log("Обновление сетки по кнопке");
        generateGrid(currentPolygon);
    } else {
        updateStatus('Нет полигона для генерации сетки', true);
    }
});

// Обработчик изменения размеров ячеек
document.getElementById('cell-width').addEventListener('change', function() {
    if (currentPolygon) {
        generateGrid(currentPolygon);
    }
});

document.getElementById('cell-height').addEventListener('change', function() {
    if (currentPolygon) {
        generateGrid(currentPolygon);
    }
});

// Обработчик изменения типа проекции
document.getElementById('projection-type').addEventListener('change', function() {
    if (currentPolygon) {
        generateGrid(currentPolygon);
    }
});

// Кнопка очистки карты
document.getElementById('clear-map').addEventListener('click', clearMap);

// Добавление элементов управления масштабом (+/-)
L.control.zoom({
    position: 'bottomright'
}).addTo(map);

// Добавление масштабной линейки
L.control.scale({
    metric: true,
    imperial: false,
    position: 'bottomleft'
}).addTo(map);

// Геолокация для установки начального положения карты
if (navigator.geolocation) {
    navigator.geolocation.getCurrentPosition(function(position) {
        map.setView([position.coords.latitude, position.coords.longitude], 14); // Увеличиваем начальный зум
    }, function() {
        // В случае ошибки геолокации, карта останется с начальными координатами
    });
}