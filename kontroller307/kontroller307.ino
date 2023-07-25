#define MOTOR1_STEP 4//пины движка
#define MOTOR1_DIR 5
#define MOTOR1_EN 6
#define KONCEVIK1 8 //концевики
#define KONCEVIK2 9
#define KONCEVIK3 10
#define ENC1_a 2 // линейный энкодер
#define ENC1_b 7
#define ENC2_a 3 //радиальный энкодер
#define ENC2_b 11
#define ENC2_z 12

#include <GyverTimers.h>
#include <GyverStepper2.h>

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;
GStepper2<STEPPER2WIRE> motor1(3200, MOTOR1_STEP, MOTOR1_DIR, MOTOR1_EN); //шагов/оборот, step, dir, en

uint64_t sendTimer; //таймер отправки sendData()
uint8_t dataIn[4] = {0}; //массив команды
uint8_t dataOut[11] = {0}; //массив данных на отправку
uint16_t targetPos = 0; // целевая позиция в ед. энкодера (50/мм)

uint8_t koncevik1Counter, koncevik2Counter;  //буфер для борьбы с помехами на концевике
bool koncevik1State, koncevik2State; //состояние концевика
bool knowHome; //известна ли позиция домашней точки

bool haveTarget = false; //переменная для подстройки позиции

volatile uint16_t encoderLinPos = 0; // счётчик энкодеров
volatile uint16_t encoderRollPos = 0;

void sendData() {
  int16_t adc0, adc1, adc2, adc3;
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/false); //отправка команды АЦП для начала измерения
  while (!ads.conversionComplete()) {//запрос проверка готово ли измерение
    delayMicroseconds(500);//задержка чтобы не отправлять запрос постоянно
  }
  adc0 = ads.getLastConversionResults(); //прочитать последнее измерение

  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_1, false);
  while (!ads.conversionComplete()) {
    delayMicroseconds(500);
  }
  adc1 = ads.getLastConversionResults();

  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_2, false);
  while (!ads.conversionComplete()) {
    delayMicroseconds(500);
  }
  adc2 = ads.getLastConversionResults();

  dataOut[0] = 3; //метка данных

  dataOut[1] = adc0 >> 8; //верхний байт
  dataOut[2] = adc0; //нижний байт

  dataOut[3] = adc1 >> 8;
  dataOut[4] = adc1;

  dataOut[5] = adc2 >> 8;
  dataOut[6] = adc2;

  dataOut[7] = adc3 >> 8;
  dataOut[8] = adc3;

  dataOut[9] = encoderLinPos >> 8; //верхний байт энкодера
  dataOut[10] = encoderLinPos;

  Serial.write(dataOut, 11); //отправить
}


//прерывание по фронту канала А энкодера
//если канал Б низкого уровня, то движемся вперёд
//Если высокого, то назад
void encoder_lin_isr() {
  if  (digitalRead(ENC1_b) == LOW) {
    encoderLinPos++;
  } else {
    if (encoderLinPos == 0) encoderLinPos = 1; //чтобы не перейти через 0
    encoderLinPos--;
  }
}


//аналогично линейному энкодеру
void encoder_roll_isr() {
  if  (digitalRead(ENC1_b) == HIGH) {
    encoderRollPos++;
  } else {
    if (encoderRollPos == 0) encoderRollPos = 1;
    else encoderRollPos--;
  }
}

//концевик 1, задний
void concevik1() {
  if (!koncevik1State) { //выполняется однократно, когда состояние false, а внутри изменится на true
    motor1.brake(); //резко остановить двигатель
    motor1.setCurrent(0); //обнулить счётчик шагов
    koncevik1State = true; //состояние концевика true
    haveTarget = false; //должно отключать движение к цели, но хз
    knowHome = true; //теперь знаем домашнюю точку
    delay(100); //задержка чтобы штанга остановилась
    encoderLinPos = 0; //обнуление энкодера
  }
}

void concevik2() {
  if (!koncevik2State) {
    motor1.brake(); //резко остановить двигатель
    koncevik2State = true; //аналогично
    haveTarget = false;
  }
}

void motorStop() {
  motor1.stop(); //плавная остановка двигателя с ускорением
}

//движение вперёд
void motorFwd(uint16_t spd) {
  haveTarget = false; //отмена движения к цели
  if (!koncevik2State) { //если не нажат передний концевик
    int s = spd;
    motor1.stop(); //остановить двигатель(костыль)
    motor1.setAcceleration(4000); //установить ускорение 4000 шаг/сек
    motor1.setMaxSpeed(s); //установить макс скорость
    motor1.setTarget(100000, RELATIVE); //установить целевую координату(немного костыль, т к координата расположена дальше концевика
    //просто разгоняемся и едем
  }
}


//аналогично движению вперёд
void motorBckwd(uint16_t spd) {
  haveTarget = false;
  if (!koncevik1State) {
    int s = spd;
    motor1.stop();
    motor1.setAcceleration(4000);
    motor1.setMaxSpeed(s);
    motor1.setTarget(-100000, RELATIVE);
  }
}


//движение к указанной точке
void motorMoveToPos(int pos) {
  motor1.stop();
  motor1.setAcceleration(6000);
  motor1.setMaxSpeed(4000);
  motor1.setTarget(pos);
}

//подстройка штанги относительно текущего положения и целевой точки
void motorMoveToPosRel(int pos) {
  motor1.stop();
  motor1.setAcceleration(100);
  motor1.setMaxSpeed(100);
  motor1.setTarget(pos, RELATIVE);
}


//едем домой
void motorHome() {
  haveTarget = false;
  if (!koncevik1State) {
    motor1.stop();
    if (knowHome) { //если известна домашняя точка, то едем чуть дальше дома до концевика
      motor1.setAcceleration(4000);
      motor1.setMaxSpeed(1600);
      motor1.setTarget(-100);
    } else { // иначе едем медленно назад, пока не упрёмся в концевик
      motor1.setAcceleration(2000);
      motor1.setMaxSpeed(800);
      motor1.setTarget(-100000, RELATIVE);
    }
  }
}


//обработчик шагов двигателя в прерывании, чтобы не было провалов в движении/ускорении
ISR(TIMER1_A) {
  motor1.tick();
}

void setup(void)
{
  Serial.begin(115200, SERIAL_8E1);
  ads.setGain(GAIN_TWOTHIRDS); // устанавливаем макс диапазон напряжений АЦП, 1 единица равна 0.1875mV
  ads.setDataRate(RATE_ADS1115_64SPS);//устанавливаем частоту дискретизации 8, 16, 32, 64, 128, 250, 475, 860 ГЦ
  // но она работает с меньшей частотой посему-то, мб АЦП китайский

  ads.begin();//база
  Wire.setClock(1000000);//установка скорости I2C чтобы быстро перекидывать байты

  Timer1.setPeriod(16); //16 uS //установка периода прерывания по таймеру в 16 мкс Меньше - не работает
  Timer1.enableISR(); //разрешение прерывания по таймеру

  pinMode(KONCEVIK1, INPUT_PULLUP);  //подтяжка контактов концевиков вверх
  pinMode(KONCEVIK2, INPUT_PULLUP);
  pinMode(KONCEVIK3, INPUT_PULLUP);

  pinMode(MOTOR1_STEP, OUTPUT); //выходы шаговика
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR1_EN, OUTPUT);

  pinMode(ENC1_a, INPUT); //А энкодера
  pinMode(ENC1_b, INPUT); //В энкодера
  attachInterrupt(0, encoder_lin_isr, RISING); //pin 2 //прерывание по каналу А энкодера
  attachInterrupt(1, encoder_roll_isr, FALLING); //pin 3 //для второго энкодера

  motor1.autoPower(true); //Выключение питания двигателя если никуда не едем
  motor1.reverse(true); //Развернуть направление вращения
}

void loop(void)
{
  //проверка состояния концевика
  if (digitalRead(KONCEVIK1) == 0) { //если 0 - то должен быть замкнут
    koncevik1Counter++;              // но это могут быть помехи, поэтому ждём пока не будет 3 низких уровня подряд
    if (koncevik1Counter > 3) concevik1(); //если больше 3 то точно замкнут, вызываем обработчик концевика
  } else {
    koncevik1Counter = 0; //если пришёл высокий уровень, то это помехи, поэтому концевик не нажат, обнуляем счётчик
    koncevik1State = false;
  }

  if ((digitalRead(KONCEVIK2) == 0)/* || (digitalRead(KONCEVIK3) == 0)*/) { //аналогично
    koncevik2Counter++;
    if (koncevik2Counter > 3) concevik2();
  } else {
    koncevik2Counter = 0;
    koncevik2State = false;
  }

  if (haveTarget && (motor1.getStatus() == 0)) { //если пришла целевая точка, то have target = true и ждём пока двигатель остановится
    motorMoveToPosRel((targetPos - encoderLinPos)); //отправляем новую команду на перемещение для подстройки положения
    if (abs(targetPos - encoderLinPos) < 10) { //гистерезис
      haveTarget = false; //приехали в нужную точку, отключаем следование к цели
      motor1.setCurrent(encoderLinPos * 0.87); //перевод значений линейного энкодера в шаги двигателя
    }                                       //работает только с такой скоростью и ускорением
  }

  if (Serial.available()) { //если пришло сообщение
    delay(2); //Ждём чтобы всё дошло
    Serial.readBytes(dataIn, 4);// посылка размером 4 байта
    if (dataIn[0] == 1) //1 байт -- адресат, если 1 - то рулим двигателем 1
    {
      if (dataIn[1] == 0) { //2 байт равен 0 -- команда остановки
        motorStop();
      } else if (dataIn[1] == 1) { //равен 1 -- едем вперёд со скоростью dataIn[2]+dataIn[3]
        motorFwd(dataIn[2] * 256 + dataIn[3]);
      } else if (dataIn[1] == 2) {//равен 2 -- едем назад со скоростью dataIn[2]+dataIn[3]
        motorBckwd(dataIn[2] * 256 + dataIn[3]);
      } else if (dataIn[1] == 4) {//равен 4 -- едем домой, ищем концевик
        motorHome();
      } else if (dataIn[1] == 8) {//равен 8 -- едем в точку dataIn[2]+dataIn[3] это расстояние в миллиметрах*100
        targetPos = (dataIn[2] * 128 + dataIn[3] / 2); //300 mm = 15000 linear
        int temp = targetPos * 0.87; //приведение значений энкодера к шагам
        motorMoveToPos(temp); //едем
        haveTarget = true; //движемся к цели
      }
    }
  }

  if (millis() - sendTimer > 50) { //таймер на отправку
    sendData(); //отправляем
    sendTimer = millis();//обновляем таймер
  }
}
