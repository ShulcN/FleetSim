#!/usr/bin/env python3
"""
Скрипт для масштабирования DXF файла.
Уменьшает/увеличивает все геометрические объекты в DXF файле.
"""

import re
import sys
import os
from pathlib import Path

def scale_dxf_file(input_file, output_file, scale_factor):
    """
    Масштабирует DXF файл.
    
    Args:
        input_file: путь к исходному DXF файлу
        output_file: путь к выходному DXF файлу
        scale_factor: коэффициент масштабирования (<1 - уменьшение, >1 - увеличение)
    """
    
    if not os.path.exists(input_file):
        print(f"Ошибка: Файл {input_file} не найден!")
        return False
    
    # Регулярные выражения для поиска координат и числовых параметров
    # Паттерны для различных DXF кодов, содержащих координаты/размеры
    coordinate_codes = {10, 11, 12, 13, 20, 21, 22, 23, 30, 31, 32, 33, 40, 41, 42, 43, 50, 51}
    
    lines_to_scale = []
    lines_to_skip = set()
    
    # Специальные DXF коды, которые НЕ нужно масштабировать
    skip_codes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 62, 70, 90, 100, 370, 999}
    
    try:
        with open(input_file, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
        
        scaled_lines = []
        i = 0
        
        while i < len(lines):
            line = lines[i].strip()
            
            # Проверяем, является ли строка DXF кодом
            if line.isdigit():
                code = int(line)
                
                # Следующая строка - значение
                if i + 1 < len(lines):
                    value_line = lines[i + 1].strip()
                    
                    # Масштабируем координаты и размеры
                    if code in coordinate_codes:
                        try:
                            # Пробуем преобразовать в число
                            num_value = float(value_line)
                            # Масштабируем
                            scaled_value = num_value * scale_factor
                            scaled_lines.append(f"{code}\n")
                            scaled_lines.append(f"{scaled_value}\n")
                            i += 2
                            continue
                        except ValueError:
                            # Не число, оставляем как есть
                            pass
                    
                    # Пропускаем масштабирование для специальных кодов
                    elif code in skip_codes:
                        pass
                    
                    # Для всех остальных числовых значений тоже масштабируем?
                    # Нет, только координаты и размеры
                
            # Если не нашли координату, просто копируем строки
            scaled_lines.append(lines[i])
            i += 1
        
        # Дополнительная обработка: масштабирование чисел в строках TEXT (высота текста)
        # и других параметров, которые могли быть пропущены
        final_lines = []
        i = 0
        
        while i < len(scaled_lines):
            line = scaled_lines[i].rstrip('\n')
            
            # Обработка высоты текста (код 40)
            if line == '40' and i + 1 < len(scaled_lines):
                next_line = scaled_lines[i + 1].rstrip('\n')
                try:
                    height = float(next_line)
                    # Масштабируем высоту текста
                    scaled_height = height * scale_factor
                    final_lines.append('40\n')
                    final_lines.append(f'{scaled_height}\n')
                    i += 2
                    continue
                except ValueError:
                    pass
            
            # Обработка кода 41 (масштаб текста по X)
            if line == '41' and i + 1 < len(scaled_lines):
                next_line = scaled_lines[i + 1].rstrip('\n')
                try:
                    text_scale = float(next_line)
                    scaled_text_scale = text_scale * scale_factor
                    final_lines.append('41\n')
                    final_lines.append(f'{scaled_text_scale}\n')
                    i += 2
                    continue
                except ValueError:
                    pass
            
            final_lines.append(scaled_lines[i])
            i += 1
        
        # Масштабируем системные переменные $EXTMIN, $EXTMAX и другие
        final_lines2 = []
        i = 0
        
        while i < len(final_lines):
            line = final_lines[i].rstrip('\n')
            
            # Масштабируем $EXTMIN, $EXTMAX
            if line == '$EXTMIN' or line == '$EXTMAX' or line == '$INSBASE':
                final_lines2.append(final_lines[i])
                i += 1
                # Следующие строки - координаты X, Y, Z с кодами 10,20,30
                coords_found = 0
                while coords_found < 3 and i < len(final_lines):
                    code_line = final_lines[i].rstrip('\n')
                    if code_line in ['10', '20', '30']:
                        final_lines2.append(final_lines[i])
                        i += 1
                        if i < len(final_lines):
                            try:
                                val = float(final_lines[i].rstrip('\n'))
                                final_lines2.append(f'{val * scale_factor}\n')
                                i += 1
                            except ValueError:
                                final_lines2.append(final_lines[i])
                                i += 1
                        coords_found += 1
                    else:
                        final_lines2.append(final_lines[i])
                        i += 1
                continue
            
            final_lines2.append(final_lines[i])
            i += 1
        
        # Записываем результат
        with open(output_file, 'w', encoding='utf-8') as f:
            f.writelines(final_lines2)
        
        print(f"✅ Файл успешно масштабирован!")
        print(f"   Коэффициент: {scale_factor}")
        print(f"   Исходный файл: {input_file}")
        print(f"   Выходной файл: {output_file}")
        
        # Вычисляем примерные новые размеры
        old_extmax = None
        new_extmax = None
        
        with open(output_file, 'r', encoding='utf-8') as f:
            content = f.read()
            match = re.search(r'\$EXTMAX\s+10\s+([\d.]+)\s+20\s+([\d.]+)', content)
            if match:
                old_match = re.search(r'\$EXTMAX\s+10\s+([\d.]+)\s+20\s+([\d.]+)', 
                                    open(input_file, 'r', encoding='utf-8').read())
                if old_match:
                    print(f"   Старые габариты: X={float(old_match.group(1)):.2f}, Y={float(old_match.group(2)):.2f}")
                    print(f"   Новые габариты: X={float(match.group(1)):.2f}, Y={float(match.group(2)):.2f}")
        
        return True
        
    except Exception as e:
        print(f"❌ Ошибка при обработке файла: {e}")
        return False


def main():
    """Основная функция"""
    
    # Настройки по умолчанию
    input_file = "input.dxf"  # Исходный файл
    output_file = "output_scaled.dxf"  # Выходной файл
    scale_factor = 0.2  # Уменьшение в 5 раз (1/5 = 0.2)
    
    # Проверяем аргументы командной строки
    if len(sys.argv) > 1:
        input_file = sys.argv[1]
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    if len(sys.argv) > 3:
        try:
            scale_factor = float(sys.argv[3])
        except ValueError:
            print(f"Ошибка: Неверный коэффициент масштабирования '{sys.argv[3]}'")
            print("Используется значение по умолчанию: 0.2")
    
    print("=" * 60)
    print("МАСШТАБИРОВАНИЕ DXF ФАЙЛА")
    print("=" * 60)
    print(f"Входной файл: {input_file}")
    print(f"Выходной файл: {output_file}")
    print(f"Коэффициент: {scale_factor} {'(уменьшение)' if scale_factor < 1 else '(увеличение)'}")
    print("=" * 60)
    
    # Масштабируем файл
    success = scale_dxf_file(input_file, output_file, scale_factor)
    
    if success:
        print("\n✨ Скрипт завершил работу успешно!")
    else:
        print("\n❌ Скрипт завершил работу с ошибкой!")
        sys.exit(1)


if __name__ == "__main__":
    main()
