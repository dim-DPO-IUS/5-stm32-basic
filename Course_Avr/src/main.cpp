#include <Arduino.h>

// Настройки фильтра
#define SMOOTHING_FACTOR 0.1    // Коэффициент сглаживания (0-1)
#define SEND_INTERVAL 100       // Интервал отправки данных (мс)
#define CHANGE_THRESHOLD 5      // Порог значимого изменения

// Глобальные переменные
float smoothedValue = 0;
unsigned long lastSendTime = 0;
int lastSentValue = 0;

void setup() {
  Serial.begin(115200);
  
  // Инициализация начального значения
  smoothedValue = analogRead(A0);
  lastSentValue = (int)smoothedValue;
}

void loop() {
  // Чтение текущего значения с потенциометра
  int rawValue = analogRead(A0);
  
  // Экспоненциальное сглаживание для устранения дребезга
  smoothedValue = smoothedValue * (1 - SMOOTHING_FACTOR) + rawValue * SMOOTHING_FACTOR;
  int currentValue = (int)smoothedValue;
  
  // Получение текущего времени
  unsigned long currentTime = millis();
  
  // Отправка данных по интервалу И при значимом изменении
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    // Проверка на значимое изменение
    if (abs(currentValue - lastSentValue) >= CHANGE_THRESHOLD) {
      Serial.println(currentValue);
      lastSentValue = currentValue;
      lastSendTime = currentTime;
    }
  }
}