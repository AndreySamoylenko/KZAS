# Пояснения к программам

«main.py» — это вспомогательная программа, исполняемая pyboard, которая принимает сообщения от raspberry и преобразует их в угол сервопривода и скорость двигателя, а также зажигает RGB светодиод.
Программы "qualification.py", "final1.py", "final2.py" загружаются на raspberry и предназначены для выполнения заданий.
Каждую из них можно разделить на 3 этапа работы:

1. ожидание кнопки
2. движение
3. финиш

(в программе этапы определяются переменной «state»)

На первом этапе raspberry отправляет сообщение «999999999999999999999$», которое получает pyboard, параллельно отправляя показания кнопки.
Когда кнопка нажата, pyboard отправляет «1 $».
raspberry принимает это сообщение и переходит в состояние движения.

В "qualification.py" второй этап — это проезд с помощью пропорционально-дифференциального регулятора по двум датчикам, с параллельным считыванием так называемых перекрёстков (оранжевая и синяя линии).
ПД-регулятор сначала вычисляет отклонение (e=dat1-dat2), а затем на его основе вычисляет управляющее воздействие (u = (e * kp + (e - e_old) * kd) (e_old - предыдущее отклонение), это управляющее воздействие отправляется pyboard. После того как число перекрёстков становится равно 12, робот проезжает ещё половину зоны и останавливается.

"final1.py" работает аналогично "qualification.py", но выполняет другие задачи, например объезд знаков.
На этапе движения на экране появляется новый датчик (хотя на самом деле два), который считывает знаки и возвращает их цвет и положение на камере.
И если робот видит знак, то вычисляет новое отклонение (e = (240 + green_pos_x * 1,3) - green_pos_y (green_pos_y — позиция знака по y, а green_pos_x — позиция по x)), в этом подсчёте не учитываются черные линии.

«final2.py» — это улучшенная версия «final1.py».
В основном, добавлена улучшенная перспектива, а также полезные алгоритмы, позволяющие улучшить результат, например посредством ускорения в зонах без знаков.

Все предыдущие программы предназначались для прохождения трассы, последующие программы нам необходимы для других целей.

«RobotAPI.py» создает класс, отвечающий за чтение камеры и в принципе работу raspberry и возможность подключения к ней.

«start_robot.py» это интерфейс для «RobotAPI.py», который упрощает нам использование робота.
Когда компьютер подключен к raspberry по IP с использованием сети Wi-Fi, в окне, созданном программой, находятся 6 кнопок: «load start», «start», «stop», «raw», «video» и «connect to robot».
Прежде всего вам необходимо подключиться к вашему роботу, для это необходимо нажать на кнопку «connect to robot» и выбрать своего робота (предварительно необходимо подключиться к Wi-Fi нашего робота и скопировать его IP в start_robot.py строка 2379).
После этого загружаем на raspberry нужную программу (кнопка "load start"). 
Если после этого вы изменили программу и хотите её загрузить, вы можете перезапустить ее кнопкой "start".
Кнопкой «video» вы можете вывести видео с камеры робота на экран вашего ноутбука в отдельном окне.
В этом же окне будет отображаться телеметрия.
Кнопка «raw» запускает тестовый файл, отображающий просто видео с камеры без телеметрии.
Кнопка «stop» останавливает запущенную программу и вывод камеры.


# Подключаемся к pyboard

Для подключения к pyboard необходимо подключить micro usb кабель к компьютеру, и открыть pyboard как флешку в проводнике.
Во вкладке PYBFLASH будет два файла «main.py» и "boot.py". Файл «main.py» — это та программа, которую будет выполнять pyboard.
Скопируйте этот файл в pycharm, при необходимости отредактируйте его и скопируйте обратно в pyboard.

# Подключение к raspberry pi

Сначала нужно включить питание робота и дождаться, пока светодиод не загорится белым.
Это означает, что raspberry готова к работе и включила точку доступа.
После этого ищем сеть нашей raspberry и подключаемся к ней.
Как только вы подключитесь к сети, вам нужно запустить программу «start_robot.py» на вашем компьютере.
Выберите своего робота.
Нажимаем «load start» и в окне с файлами выбираем подходящий.
Всё, ваша программа загружена и если на pyboard горят синий и желтый светодиоды, значит он успешно получает сообщения от raspberry.

# Запуск программы на Raspberry Pi.

После того как вы выбрали программу в «start_robot.py», программу можно запустить, нажав кнопку на роботе.
Но если вы выключили и включили робота, то запустится не ваша программа.
В этом случае запускается программа, которая прописана в "autostart.py".
Поэтому можно записать нужную программу в "autostart.py" и перезапускать программу через "start_robot.py" будет не нужно).

# Если что-то пошло не так

Если робот не работает, то рекомендуется выполнить следующие действия.
В очень вероятной ситуации, когда робот не подключается, сначала подключитесь к Wi-Fi сети вашей raspberry.
После этого откройте свойства сети и найдите там IP.
Этот IP нужно закинуть в "start_robot.py"(строка 2379).

Если робот работает неправильно, проверьте напряжение на вольтметре. Если оно ниже 5В, немедленно меняйте батарейки (рекомендуется менять батарейки, как только напряжение упадет до 7В).

Если ни на raspberry ни на pyboard не горит красный светодиод значит на них нет питания, в таком случае рекомендуется проверить подключение.



