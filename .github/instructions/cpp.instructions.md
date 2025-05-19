---
applyTo: '**/*.{cpp,hpp}'
---
# Стандарты кодирования и соглашения для C++ (ROS 2 Jazzy)

- Стандарт: **C++17**
- Стиль: [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) с модификациями ROS 2:
    - Максимальная длина строки: **100 символов**
    - Заголовочные файлы: `.hpp`, исходники: `.cpp`
    - Глобальные переменные: `g_<имя>` (snake_case)
    - Имена функций и методов: предпочтительно `snake_case` (как в std), классы — `CamelCase`
    - Всегда используйте фигурные скобки после if/else/for/while, даже для одной строки
    - Для функций, классов, enum, struct — открывающая скобка на новой строке
    - Для if/else/while/for — скобка на той же строке (cuddled)
    - Используйте `///` и `/** ... */` для документации, `//` для обычных комментариев
    - Исключения разрешены, но избегайте их в деструкторах
    - Boost использовать только при крайней необходимости
    - Не используйте пробел перед `public:`, `private:`, `protected:`
    - Не добавляйте пробелы в шаблонах: `set<list<string>>`
    - Выравнивание указателей: `char *c;`, а не `char* c;`
    - Для длинных вызовов функций — переносите на новую строку после открывающей скобки, с отступом 2 пробела

## Линтеры и автоформатирование
- Используйте:
    - `ament_clang_format` ([.clang-format](https://github.com/ament/ament_lint/blob/jazzy/ament_clang_format/ament_clang_format/configuration/.clang-format))
    - `ament_cpplint`
    - `ament_uncrustify` ([ament_code_style.cfg](https://github.com/ament/ament_lint/blob/jazzy/ament_uncrustify/ament_uncrustify/configuration/ament_code_style.cfg))
    - `ament_cppcheck` для статического анализа
- Пример автоформатирования:

```bash
ament_clang_format --reformat
```
