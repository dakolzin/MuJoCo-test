#!/usr/bin/env python3
import subprocess
import sys

def main():
    print("Нажмите Enter, чтобы вызвать сервис /next_grasp. Для выхода нажмите Ctrl+C.")
    try:
        while True:
            input()  # Ожидание нажатия Enter
            command = 'ros2 service call /next_grasp std_srvs/srv/Empty "{}"'
            print("Выполняется команда:", command)
            subprocess.run(command, shell=True)
    except KeyboardInterrupt:
        print("\nВыход. До свидания!")
        sys.exit(0)

if __name__ == '__main__':
    main()
