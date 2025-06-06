# STM32-Curve25519-UART-ESP8266

В рамках проекта разрабатывается прототип системы безопасного обмена ключами на основе Curve25519 для STM32 и ESP8266 с использованием UART.

## Описание

На данный момент реализована частичная функциональность, которая позволяет:
- Отправлять данные по ESP8266.
- Отправлять данные по UART на ПК.
- Генерируется публичный ключ на основе приватного(пока фиксированное значение).

Позже планурется осужествлять обмен ключами между esp8266 и stm32.

## Сборка

### stm32
Для сборки проекта используется **libopencm3** для работы с микроконтроллером STM32. Проект использует Makefile для управления процессом сборки.

1. Склонируйте репозиторий.
   ```bash
   git clone --recursive git@github.com:RenChan18/STM32-Soil-Moisture-Monitor-with-Local-Server.git
   ```
2. Убедитесь, что у вас установлены следующие зависимости:
   - GCC для ARM (arm-none-eabi)
   - Make
3. Соберите проект с помощью команды:
   ```bash
   make
   ```

### esp8266
Работа осуществляется с использованием фреймворка [ESP8266_RTOS_SDK](https://github.com/espressif/ESP8266_RTOS_SDK).
Следуйте приведённым шагам, чтобы правильно настроить и использовать SDK.

#### Установка и настройка виртуального окружения
Удобно использовать виртуальное окружение для изоляции зависимостей.
1. Создайте виртуальное окружение:
```bash
python3.11 -m venv ~/esp8266-venv
```
2. Активируйте виртуальное окружение:
```bash
source ~/esp8266-venv/bin/activate
```

#### Клонирование ESP8266_RTOS_SDK и настройка окружения
1. Склонируйте репозиторий SDK:
```bash
git clone --recursive https://github.com/espressif/ESP8266_RTOS_SDK.git
```
2. Далее пожалйста следуйте инструкциям [ESP8266_RTOS_SDK](https://github.com/espressif/ESP8266_RTOS_SDK).

### Сборка прошивки
Соберите проект, выполнив команду:
```bash
make
```

### Загрузка прошивки на ESP8266 (прошивка)
Загрузите собранную прошивку на устройство:
```bash
make flash
```

### Мониторинг вывода (запуск)
Чтобы просмотреть вывод с устройства после прошивки, используйте команду:
```bash
make monitor
```

- Для выхода из монитора нажмите `Ctrl + ]`.

## Подключения

- **TTL-USB преобразователь**: Обеспечивает последовательную связь между STM32 и компьютером или другими устройствами.

#### **STM32F411VET & TTL-USB преобразователь (Последовательная связь)**
| **STM32F411VET** | **TTL-USB преобразователь** |
| :--------------: | :-----------------------: |
|       TX (PA9)    |          RX               |
|       RX (PA10)   |          TX               |
|       GND         |          GND              |


