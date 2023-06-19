# О содержании репозитория

В папке ![технический журнал](https://github.com/AndreySamoylenko/KZAS/blob/main/технический%20журнал) находятся три документа, в них рассмотрено устройство робота и логика прохождения трассы. 

В файле ![Обсуждение системы перемещения.pdf](https://github.com/AndreySamoylenko/KZAS/blob/main/технический%20журнал/Обсуждение%20системы%20перемещения.pdf) рассмотрены некоторые 3D модели и их назначение, а также некоторые аспекты выбора комплектующих.

В файле ![Обсуждение системы питания и сенсоров.pdf](https://github.com/AndreySamoylenko/KZAS/blob/main/технический%20журнал/Обсуждение%20системы%20питания%20и%20сенсоров.pdf) представлены сравнения электрических компонентов, и объяснён выбор тех моделей которые вошли в конечную схему. В этом же файле вы можете ознакомиться с логической и принципиальной схемами, которые показывают взаимодействия компонентов и устройство электрической схемы робота.

В файле ![Обсуждение системы распознавания препятствий.pdf](https://github.com/AndreySamoylenko/KZAS/blob/main/технический%20журнал/Обсуждение%20систем%20распознавания%20препятствий.pdf) показаны и разобраны некоторые алгоритмы, и объяснены части кода программ из папки ![программы](https://github.com/AndreySamoylenko/KZAS/blob/main/программы). В этом документе не разобраны программы не отвечающие за распознавание препятствий.

Папка ![программы](https://github.com/AndreySamoylenko/KZAS/blob/main/программы) содержит все программы использованные нами в процессе разработки

В папке ![видео](https://github.com/AndreySamoylenko/KZAS/blob/main/видео) находится текстовый файл с ссылкой на видео.

Папка  ![Фотографии робота](https://github.com/AndreySamoylenko/KZAS/blob/main/Фотографии%20робота) содержит 11 фотграфий, 10 фото робота и 1 фото команды.

В папке ![3D модели](https://github.com/AndreySamoylenko/KZAS/blob/main/3D%20модели) вы можете найти все 3д модели деталей, которые мы печатали.


# Пояснения к программам 

Здесь будет представлено краткое описание программ.

![RobotAPI.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/RobotAPI.py) - это класс используемый программой ![start_robot.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/start_robot.py). Он необходим для загрузки программы на распберри, и получения изображения с камеры через Wi-Fi.

![start_robot.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/start_robot.py) программа упрощающая загрузк 

 

![main.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/main.py) это вспомогательная программа, исполняемая микроконтроллерром. 
Сама прорамма выглядит примерно так: 
1) принять сообщение от raspberry  
2) расшифровать его и получить целевой угол сервопривода, скорость мотора и цвет RGB светодиода
3) зажечь светодиод нужным цветом
4) запустить мотор с нужной скоростью
5) повернуть сервопривод на целевой угол
6) повторить


Программы ![qualification](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/qualification.py), ![final.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/final.py) загружаются на raspberry pi и имеют схожую структуру. 

Все их можно разделить на 3 этапа работы: 

1. ожидание кнопки 

2. движение 

3. остановка 

(в программе этапы определяются переменной "state") 

На первом этапе raspberry pi отправляет сообщение "999999999999999999999$", которое pyboard считывает и ждет изменения значения, параллельно отправляя показания кнопки. 

Когда кнопка нажата, pyboard отсылает сообщение: "1$". 

raspberry получает его и начинает движение.   

В ![qualification](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/qualification.py), второй этап - прохождение с помощью пропорционально-дифференциального регулятора с двумя датчиками и с параллельным считыванием так называемых пере (оранжевая и синяя линии). 

ПД-регулятор сначала вычисляет отклонение (e=dat1-dat2) исходя из показаний датчиков бортика, а затем на его основе вычисляет управляющее воздействие (u = (e * kp + (e - e_old) * kd) (e_old - предыдущее отклонение). 

![final.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/final.py) работает аналогично ![qualification](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/qualification.py) но в него добавлены функции обхода знаков. 

На этапе прохождения трассы на экране появляются два больших датчика, которые ищут знаки и возвращают их цвет и положение на камере. 
 
И если робот замечает знак, то вычисляет новое отклонение отнцосительно него, игнорируя черные линии. 
 
Все предыдущие программы были для прохождения трассы, последующие программы понадобятся нам для других целей. 

![RobotAPI.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/RobotAPI.py) создает класс, отвечающий за считывание изображения с камеры. 

![start_robot.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/start_robot.py) используя ![RobotAPI.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/RobotAPI.py) упрощает использование робота. Эта программа создаёт меню, с помощью которого вы можете удобно загрузить программу на робота, запустить или остановить её в любой момент. 
 
В окне, созданном программой, наблюдаем 6 кнопок: "load start", "start", "stop", "raw" , "video" and "connect to robot". 

После того как вы запустили ![start_robot.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/start_robot.py) и попробуете что-нибудь сделать, программа выведет сообщение о том, что сначала нужно выбрать робота Это означает, что прежде чем что-то загружать, нужно знать, куда это загружать. 
В этом случае вам необходимо нажать на кнопку "подключиться к роботу" и выбрать своего робота (предварительно необходимо подключиться к Wi-Fi сети вашего робота). 
После этого заливаем на raspberry pi нужную программу (кнопка " load start ") или, если она уже загружена, перезапускаем ее кнопкой "start". 
 
Кнопкой "video" вы можете вывести видео с камеры робота на экран вашего ноутбука в отдельном окне. 
 
В этом же окне будет отображаться телеметрия. 
 
Кнопка "raw" запускает тестовый файл, который выводит изображение с камеры без телеметрии. 
 
Кнопка "stop" останавливает запущенную программу и вывод камеры (при остановке программы робот не остановится, он продолжит ехать с последней заданной скоростью и сохранит угол поворота). 
 
# Подключаемся к pyboard 

Для подключения к pyboard необходимо с помощью micro usb кабеля подключить pyboard к компьютеру. После этого в проводнике появится устройство с названием PYBFLASH. 

В на этом устройстве будут два файла ![main.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/main.py) и boot.py. Файл ![main.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/main.py) - это та программа, которую будет исполнять pyboard. 

Вы можете скопировать её в свою среду разработки и изменить, если это необходимо, после этого вам нужно заменить файл ![main.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/main.py) в папке PYBFLASH на отредактированный. 
  
# Подключение к raspberry pi 

Сначала вам нужно включить питание робота и дождаться, пока RGB-светодиод не загорится белым цветом. 

Это означает, что raspberry pi готова к работе и включила точку доступа. 
После этого ищем сеть нашей raspberry и подключаемся к ней. 
Как только вы подключитесь к сети, вам нужно запустить программу start_robot.py на вашем компьютере. 
Далее необходимо выбрать робота. 
 
После этого нажимаем кнопку " load start " и в окне с файлами выбираем нужный. 
 
Ваша программа будет загружена и если на pyboard горят синий и желтый светодиоды, значит pyboard успешно получает сообщения от raspberry. 

# Запуск программы на raspberry pi

После того, как вы выбрали программу в ![start_robot.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/start_robot.py), ее можно запустить, нажав кнопку на роботе. 
 
Но если вы выключили и включили робота, то ваша программа не запустится. 
 
В этом случае запускается программа, которая импортирована в ![autostart.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/autostart.py). 
 
Поэтому вы можете импортировать нужную программу в ![autostart.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/autostart.py) и вам не нужно будет перезапускать программу через start_robot.py. 

# Если что-то пошло не так 

Если вы столкнулись с трудностями, или робот не работает, то рекомендуется выполнить следующие действия. 
 
Если вы не можете подключиться к роботу через ![start_robot.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/start_robot.py) используя ![RobotAPI.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/RobotAPI.py), то вам следует сначала подключиться к WiFi сети вашей raspberry, после этого открыть свойства сети и найти IP адрес сети. Копируем его и вставляем в 2379 строку ![start_robot.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/start_robot.py) используя ![RobotAPI.py](https://github.com/AndreySamoylenko/KZAS/blob/main/программы/RobotAPI.py) вместо старого IP, там же вы можете изменить название вашего робота в программе. 

Если робот работает неправильно, проверьте напряжение на вольтметре. Если оно ниже 7,5В, немедленно меняйте аккумуляторы (рекомендуется менять аккумуляторы, как только напряжение упадет до 10,5В). 
 
   
