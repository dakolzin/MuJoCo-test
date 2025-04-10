#!/bin/bash

# Сохраняем исходный каталог
ORIGINAL_DIR=$(pwd)

# Обработка сигнала прерывания Ctrl+C
trap 'echo "Прерывание! Возвращаемся в $ORIGINAL_DIR"; cd "$ORIGINAL_DIR"; exit 1' INT

cd my_sim

python3 vase.py

cd ..
