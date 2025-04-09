import json
import os

def chunkify(lst, chunk_size):
    # Функция возвращает части списка размера chunk_size
    for i in range(0, len(lst), chunk_size):
        yield lst[i:i + chunk_size]

def main():
    input_filename = "best_grasps.json"
    # Читаем данные из файла best_grasps.json
    with open(input_filename, "r", encoding="utf-8") as f:
        best_grasps = json.load(f)
    
    # Извлекаем массивы захватов из каждого объекта
    grasp_arrays = [entry["grasp_array"] for entry in best_grasps if "grasp_array" in entry]

    # Размер чанка – 17 захватов
    chunk_size = 10

    # Для каждого чанка формируем объект с нужной структурой и записываем в файл
    for index, chunk in enumerate(chunkify(grasp_arrays, chunk_size), start=1):
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
