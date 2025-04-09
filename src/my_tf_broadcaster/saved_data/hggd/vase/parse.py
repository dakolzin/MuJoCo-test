import json

def chunkify(lst, chunk_size):
    # Функция для разбиения списка lst на части (чанки) размера chunk_size
    for i in range(0, len(lst), chunk_size):
        yield lst[i:i + chunk_size]

def main():
    input_filename = "best_grasps.json"
    
    # Читаем данные из файла best_grasps.json
    with open(input_filename, "r", encoding="utf-8") as f:
        grasps = json.load(f)
    
    # Извлекаем массив параметров захвата из каждого объекта
    raw_params = [entry["raw_17_params"] for entry in grasps if "raw_17_params" in entry]

    # Задаём размер чанка – 17 захватов на файл
    chunk_size = 10

    # Разбиваем список массивов на чанки и сохраняем в файлы
    for index, chunk in enumerate(chunkify(raw_params, chunk_size), start=1):
        data = {
            "frame_index": index,
            "num_grasps": len(chunk),
            "grasps_17": chunk
        }
        output_filename = f"grasp_data_{index:06d}.json"
        with open(output_filename, "w", encoding="utf-8") as out_file:
            json.dump(data, out_file, ensure_ascii=False, indent=4)
        print(f"Записан файл {output_filename} с {len(chunk)} захватами.")

if __name__ == "__main__":
    main()
