# testing_task

При работе над тестовым заданием придерживался принципа читаемости кода, отдавая предпочтение ей, а не оптимизации.
Для примера переиспользования кода(future-proof) реализовал интерфейс класса, для возможности последующей замены алгоритма в зависимости от нужд.

Для сборки выполнить команды:
```
git clone https://github.com/shamoleg/testing_task
cd testing_task
mkdir build && cd build
cmake ..
make
```

Запуск программмы:
```
./testing_task
```

Для проверки работы ввести в терминал:
```
1
50 50 20
3
30 30 90
60 60 80
10 90 100
3
30 30 90
60 60 80
10 90 10
0

```
Ожидаемый вывод:
```
90.711
156.858
110.711
```
