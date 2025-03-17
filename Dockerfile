# Используйте ваш базовый образ
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

# Устанавливаем необходимые переменные окружения
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV ROS_DISTRO=humble

# Обновляем репозитории и устанавливаем зависимости
RUN apt-get update && apt-get install -y \
    iperf3 \
    nano \
    python3-pip \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем рабочую директорию
WORKDIR /mujoco

# Копируем только файлы setup.py и requirements.txt
COPY setup.py requirements.txt /mujoco/

# Устанавливаем пакет в editable-режиме (при условии, что setup.py корректно описывает пакет)
RUN pip3 install -e .

# Точка входа – запускаем bash
ENTRYPOINT ["/bin/bash"]
