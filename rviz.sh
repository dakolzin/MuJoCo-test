#!/bin/bash

# Сохраняем исходный каталог
ORIGINAL_DIR=$(pwd)

# Обработка сигнала прерывания Ctrl+C
trap 'echo "Прерывание! Возвращаемся в $ORIGINAL_DIR"; cd "$ORIGINAL_DIR"; exit 1' INT

cd src/

colcon build

. install/setup.bash

# Определяем режим. Если передан аргумент, используем его, иначе --vase по умолчанию.
MODE="--vase"
if [ "$1" != "" ]; then
    MODE="$1"
fi

echo "Запуск с режимом: $MODE"
ros2 launch my_tf_broadcaster view.launch.py mode:=$MODE

cd ..
