# 3d-pathfinding-JPS-
Проект сделан в рамках курса по основам и методам эвристического поиска и фокусируется на поиске путей между парами вершин на 3D-сетке без срезания углов с использованием алгоритмов A* и его модификации Jump point search (JPS). В ходе проекта сравниваются результаты работы A* и JPS на картах из Warframe, популярной многопользовательской онлайн-игры, карты получены отсюда https://movingai.com/benchmarks/voxels.html.

## Запуск

Файл test.py готов к запуску, в данном файле можно поменять названия файлов .3dmap и .3dmap.3dscen и проверить результаты на желаемой карте из директории maps. 
 
## Входные/выходные данные

Входные данные (папка maps) состоят каждый из файла .3dmap, описывающего карту, и файла .3dmap.3dscen с набором тестов для данной карты.
В первой строке файла 3dmap находятся 3 числа - размеры карты по трем измерениям, во всех последующих строках 3 числа обозначают x/y/z коодинаты препятствия.
Файл 3dmap.3dscen в каждой из третьей и последующих строк содержит x/y/z координаты начала/цели, затем длину оптимального пути и коэффциент между длиной оптимального пути и ее эвристической оценкой.




