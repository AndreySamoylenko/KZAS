# О содержании репозитория

В папке ![технический журнал](https://github.com/AndreySamoylenko/KZAS/blob/main/технический%20журнал) находятся три документа, в них рассмотрено устройство робота и логика прохождения трассы. 

В файле ![Обсуждение системы перемещения.pdf](https://github.com/AndreySamoylenko/KZAS/blob/main/технический%20журнал/Обсуждение%20системы%20перемещения.pdf) рассмотрены некоторые 3D модели и их назначение, а также некоторые аспекты выбора комплектующих.

В файле ![Обсуждение системы питания и сенсоров.pdf](https://github.com/AndreySamoylenko/KZAS/blob/main/технический%20журнал/Обсуждение%20системы%20питания%20и%20сенсоров.pdf) представлены сравнения электрических компонентов, и объяснён выбор тех моделей которые вошли в конечную схему. В этом же файле вы можете ознакомиться с логической и принципиальной схемами, которые показывают взаимодействия компонентов и устройство электрической схемы робота.

В файле ![Обсуждение системы распознавания препятствий.pdf](https://github.com/AndreySamoylenko/KZAS/blob/main/технический%20журнал/Обсуждение%20систем%20распознавания%20препятствий.pdf) показаны и разобраны некоторые алгоритмы, и объяснены части кода программ из папки ![программы](https://github.com/AndreySamoylenko/KZAS/blob/main/программы). В этом документе не разобраны программы не отвечающие за распознавание препятствий.

Папка ![программы](https://github.com/AndreySamoylenko/KZAS/blob/main/программы) содержит все программы использованные нами в процессе разработки

В папке ![видео](https://github.com/AndreySamoylenko/KZAS/blob/main/видео) находится текстовый файл с ссылкой на видео.

Папка  ![Фотографии робота](https://github.com/AndreySamoylenko/KZAS/blob/main/Фотографии%20робота) содержит 11 фотографий, 10 фото робота и 1 фото команды.

В папке ![3D модели](https://github.com/AndreySamoylenko/KZAS/blob/main/3D%20модели) вы можете найти все 3д модели деталей, которые мы печатали.

# Пояснения к программам 

Здесь будет представлено краткое описание программ.

![RobotAPI.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/RobotAPI.py) - это класс используемый программой ![start_robot.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/start_robot.py). Он необходим для загрузки программы на распберри, и получения изображения с камеры по Wi-Fi.

![start_robot.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/start_robot.py) - это программа создающая удобный интерфейс для взаимодействия с роботом (фотка). Использование этой программы будет рассмотрено подробно в разделе "подключение к распберри".

![main.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/main.py) - это программа, исполняемая микроконтроллерром. 
Сама программа выглядит примерно так: 
1) принять сообщение от raspberry  
2) расшифровать его и получить целевой угол сервопривода, скорость мотора и цвет RGB светодиода
3) зажечь светодиод нужным цветом
4) запустить мотор с нужной скоростью
5) повернуть сервопривод на целевой угол
6) повторить

Программы ![qualification](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/qualification.py), ![final.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/final.py) загружаются на raspberry pi с помощью ![start_robot.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/start_robot.py), и имеют схожую структуру. 

Все их можно разделить на 3 этапа работы (в программе этапы определяются переменной "state"): 

1) ожидание нажатия кнопки (state == 0)
2) прохождение трассы (state == 1)
3) остановка (state == 3)

(когда state == 2 робот проходит последние ползоны чтобы финишировать в центре) 

На первом этапе raspberry отправляет сообщение "999999999999999999999$", которое pyboard получает и ждет изменения значения, параллельно отправляя показания кнопки. 

Когда кнопка нажата, pyboard отсылает сообщение: "1$". 
raspberry получает это сообщение и начинает движение.   

В ![qualification](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/qualification.py), второй этап - прохождение трассы с помощью пропорционально-дифференциального регулятора с двумя датчиками и с параллельным считыванием так называемых перекрёстков (оранжевая и синяя линии). 

ПД-регулятор сначала вычисляет отклонение (e=dat1-dat2) исходя из показаний датчиков бортика, а затем на его основе вычисляет управляющее воздействие (u = (e * kp + (e - e_old) * kd)  (e_old - предыдущее отклонение). 

![final.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/final.py) работает иначе - в него добавлены функции избегания знаков. 

На кадре появляется большой датчик, который ищет знаки и возвращают их цвет и координаты на кадре, а также, в программе появляется новый ПД регулятор, и если робот замечает знак, то вычисляет новое отклонение относительно этого знака, игнорируя черные линии и используя специальный ПД регулятор для знаков. 
Также для более быстрого прохождения трассы мы добавили функцию разгона, то есть когда робот не видит знаков, его скорость растёт, а когда знак появляется на кадре скорость падает.

Также в папке ![программы](https://github.com/AndreySamoylenko/KZAS/blob/main/программы) есть файл ![autostart.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/autostart.py), в этот файл вы можете импортировать пр  

# Подключение к pyboard 

Для подключения к pyboard необходимо с помощью micro usb кабеля подключить pyboard к компьютеру. После этого в проводнике появится устройство с названием PYBFLASH. 

В на этом устройстве будут два файла main.py и boot.py. Файл ![main.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/main.py) - это та программа, которую будет исполнять pyboard. 

Вы можете скопировать её в свою среду разработки и изменить, если это необходимо, после этого вам нужно заменить файл ![main.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/main.py) в папке PYBFLASH на отредактированный, и после того как погаснет красный светодиод нужно будет нажать на кнопку RESET (RST). 
После этого pyboard перезагрузится и начнёт выполнение программы ![main.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/main.py)
  
# Подключение к raspberry pi 

Сначала вам нужно включить питание робота (или запитать распберри отдельно) и подождать какое-то время, это нужно для того чтобы на распберри запустилась операционная система, и включилась точка доступа. 
После этого ищем сеть нашей raspberry и подключаемся к ней. 
Как только вы подключитесь к сети, вам нужно запустить программу start_robot.py на вашем компьютере. 
приду домой опишу процесс с фотками
 
После этого нажимаем кнопку " load start " и в окне с файлами выбираем нужный. 
 
Ваша программа будет загружена и если на pyboard горит желтый светодиод, значит pyboard успешно получает сообщения от raspberry. 

После того, как вы выбрали программу в ![start_robot.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/start_robot.py), ее можно запустить, нажав кнопку на роботе. 
 
Но если вы выключили и включили робота, то ваша программа не запустится. 
 
В этом случае запускается программа, которая импортирована в ![autostart.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/autostart.py). 
 
Поэтому вы можете импортировать нужную программу в ![autostart.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/autostart.py) и вам не нужно будет перезапускать программу через start_robot.py. 

# Если что-то пошло не так 

Если вы столкнулись с трудностями, или робот не работает, то рекомендуется выполнить следующие действия. 
 
Если вы не можете подключиться к роботу через ![start_robot.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/start_robot.py) перечитайте раздел "подключение к raspberry pi". 

Если робот работает неправильно, проверьте напряжение на вольтметре. Если оно ниже 9 вольт, немедленно меняйте аккумуляторы (рекомендуется менять аккумуляторы, как только напряжение упадет до 10,5 вольт). 
 
   
