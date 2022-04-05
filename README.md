# drone-api
Упрощенное управление PX4+ROS  
Проект на стадии разработки

ЧИТАЙ [**WIKI**](https://github.com/boris-gu/drone-api/wiki)!!!

## Зависимости
* [ROS Noetic](http://wiki.ros.org/noetic)  
* [Catkin Command Line Tools](https://catkin-tools.readthedocs.io/en/latest/) - для сборки пакетов в python_ws
* Python3
* OpenCV-Python
* PyGeodesy - объяснение необходимости [тут](https://wiki.ros.org/mavros#mavros.2FPlugins.Avoiding_Pitfalls_Related_to_Ellipsoid_Height_and_Height_Above_Mean_Sea_Level)
* [fpv-drone](https://github.com/boris-gu/fpv-drone) - модели дронов с камерой для запуска скриптов aruco_*.py в Gazebo
* [marker-gazebo-generator](https://github.com/boris-gu/marker-gazebo-generator) - создание модели Aruco маркера для запуска скриптов aruco_*.py в Gazebo

## Работа с репозиторием
1. Установите все НЕОБХОДИМЫЕ зависимости. Если собираетесь работать в симуляторе Gazebo, добавьте модели дронов в симулятор по инструкции из [реопзитория](https://github.com/boris-gu/fpv-drone) и ArUco маркер в папку **/home/&lt;user&gt;/.gazebo/models** или **&lt;PX4-Autopilot_clone&gt;/Tools/sitl_gazebo/models**
2. Соберите пакет ROS:
    ```
    cd <DRONE_API_CLONE>/python_ws
    catkin build
    ```
3. Для запуска скрипта в Gazebo можно использовать **rosrun**, запускать командой **`<SCRIPT_PATH>/<SCRIPT NAME>`**, или использовать **roslaunch**. Файлы .launch лежат в **&lt;DRONE_API_CLONE&gt;/python_ws/launch**.
4. В процессе создания своих скриптов можно просто добавлять их в папку  **&lt;DRONE_API_CLONE&gt;.python_ws/src/test_pkg/src**, аналогично с .launch файлами.
