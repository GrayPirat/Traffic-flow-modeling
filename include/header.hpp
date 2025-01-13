#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <iomanip>
#include <numeric>
#include <map>
#include <chrono>

using namespace std;
const double M_PI = acos(-1);

// Структура для хранения данных о транспортном потоке
struct Vehicle {
    double x;
    double y;
    double speed;
    double angle;
    bool isLeader;  // Является ли автомобиль лидером пачки
    int clusterId;
};

// Структура для хранения данных сенсора
struct Sensor {
    double x;
    double y;
    double radius;
};

double barlettDistribution(double r, double q, int k) {
    if (k == 1) return 1 - r;
    if (k >= 2) return r * (1 - q) * pow(q, k - 2);
    return 0;
}

// Функция для генерации пачки автомобилей
vector<Vehicle> generateTrafficCluster(double maxX, double maxY, double minSpeed, double maxSpeed, mt19937& gen, uniform_real_distribution<>& xDist, uniform_real_distribution<>& yDist, uniform_real_distribution<>& angleDist, double leaderSpeed, double clusterProb, double clusterLengthParam, int& clusterIdCounter, double r, double q) {
    vector<Vehicle> cluster;
    uniform_real_distribution<> probDist(0, 1.0);
    int clusterLength = 0;
    double prob = 0;
    do {
        clusterLength++;
        prob += barlettDistribution(r, q, clusterLength);

    } while (probDist(gen) > prob);

    uniform_real_distribution<> speedDist(leaderSpeed, maxSpeed);


    // Генерация лидера
    Vehicle leader;
    leader.x = xDist(gen);
    leader.y = yDist(gen);
    leader.speed = leaderSpeed;
    leader.angle = angleDist(gen);
    leader.isLeader = true;
    leader.clusterId = clusterIdCounter;
    cluster.push_back(leader);

    // Генерация остальных автомобилей
    for (int i = 1; i < clusterLength; ++i) {
        Vehicle vehicle;
        vehicle.x = xDist(gen);
        vehicle.y = yDist(gen);
        vehicle.speed = speedDist(gen);
        vehicle.angle = angleDist(gen);
        vehicle.isLeader = false;
        vehicle.clusterId = clusterIdCounter;
        cluster.push_back(vehicle);
    }
    clusterIdCounter++;
    return cluster;
}

// Функция для создания сенсоров
vector<Sensor> createSensors(int numSensors, double maxX, double maxY, double sensorRadius, int numRegions) {
    vector<Sensor> sensors;
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> xDist(0, maxX);
    uniform_real_distribution<> yDist(0, maxY);

    for (int i = 0; i < numSensors; ++i) {
        Sensor sensor;
        sensor.x = xDist(gen);
        sensor.y = yDist(gen);
        sensor.radius = sensorRadius;
        sensors.push_back(sensor);
    }
    return sensors;
}

// Функция для моделирования движения
void simulateTrafficFlow(vector<Vehicle>& trafficFlow, double timeStep, double maxX, double maxY, double jamProbability, double leaderSpeed, double r, double q)
{
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> probDist(0, 1.0);
    uniform_real_distribution<> speedVariation(0.5, 1.0);

    for (auto& vehicle : trafficFlow)
    {
        double prob = probDist(gen);
        if (prob > jamProbability)
        {
            if (vehicle.isLeader) {
                vehicle.x += vehicle.speed * cos(vehicle.angle) * timeStep;
                vehicle.y += vehicle.speed * sin(vehicle.angle) * timeStep;
            }
            else {
                // Ищем лидера кластера
                for (auto& leader : trafficFlow) {
                    if (leader.isLeader && leader.clusterId == vehicle.clusterId) {
                        double leaderX = leader.x;
                        double leaderY = leader.y;
                        double dist = sqrt(pow(vehicle.x - leaderX, 2) + pow(vehicle.y - leaderY, 2));
                        // Если расстояние до лидера меньше 5, то двигаемся с его скоростью,
                        //  иначе двигаемся с нашей скоростью.
                        if (dist < 5) {
                            vehicle.x += (leader.speed * speedVariation(gen)) * cos(vehicle.angle) * timeStep;
                            vehicle.y += (leader.speed * speedVariation(gen)) * sin(vehicle.angle) * timeStep;
                        }
                        else {
                            vehicle.x += vehicle.speed * cos(vehicle.angle) * timeStep;
                            vehicle.y += vehicle.speed * sin(vehicle.angle) * timeStep;
                        }
                        break;
                    }
                }
            }
            if (vehicle.x < 0) vehicle.x = maxX;
            if (vehicle.x > maxX) vehicle.x = 0;
            if (vehicle.y < 0) vehicle.y = maxY;
            if (vehicle.y > maxY) vehicle.y = 0;
        }
    }
}

// Функция для сбора данных о трафике в сенсорах
vector<int> collectTrafficData(const vector<Vehicle>& trafficFlow, const vector<Sensor>& sensors) {
    vector<int> trafficData(sensors.size(), 0);
    for (const auto& vehicle : trafficFlow) {
        for (int i = 0; i < sensors.size(); ++i) {
            double dist = sqrt(pow(vehicle.x - sensors[i].x, 2) + pow(vehicle.y - sensors[i].y, 2));
            if (dist <= sensors[i].radius) {
                trafficData[i]++;
            }
        }
    }
    return trafficData;
}


// Функция для Бартлеттовского анализа трафика в регионе
double bartlettAnalysis(const vector<int>& trafficData, const vector<int>& regionSensorIndices) {
    if (trafficData.empty() || regionSensorIndices.empty()) return 0;
    double sum = 0.0;
    for (int index : regionSensorIndices)
    {
        sum += trafficData[index];
    }
    return sum / (double)regionSensorIndices.size();
}

// Функция для сканирования по областям (с учетом времени)
map<int, vector<double>> scanTrafficRegions(vector<Vehicle>& trafficFlow, const vector<Sensor>& sensors, int numRegions, double timeStep, int numTimeSteps, double maxX, double maxY, double jamProbability, double leaderSpeed, double r, double q)
{
    map<int, vector<double>> regionTrafficData;
    for (int regionIdx = 0; regionIdx < numRegions; ++regionIdx)
    {
        vector<double> timeSeries;
        vector<Vehicle> currentTrafficFlow = trafficFlow;
        for (int t = 0; t < numTimeSteps; ++t)
        {
            simulateTrafficFlow(currentTrafficFlow, timeStep, maxX, maxY, jamProbability, leaderSpeed, r, q);
            vector<int> trafficData = collectTrafficData(currentTrafficFlow, sensors);
            vector<int> sensorIndices;
            for (int i = 0; i < sensors.size(); ++i)
            {
                if (i % numRegions == regionIdx) {
                    sensorIndices.push_back(i);
                }
            }
            double bartlettValueForRegion = bartlettAnalysis(trafficData, sensorIndices);
            timeSeries.push_back(bartlettValueForRegion);
        }
        regionTrafficData[regionIdx] = timeSeries;
    }
    return regionTrafficData;
}


// Функция для вывода визуализации
void printVisualization(const vector<Vehicle>& trafficFlow, const vector<Sensor>& sensors, int maxX, int maxY) {
    const int gridX = 20;
    const int gridY = 10;
    vector<vector<char>> grid(gridY, vector<char>(gridX, '.')); // Создаем пустую сетку
    double scaleX = (double)gridX / maxX;
    double scaleY = (double)gridY / maxY;
    // Добавляем сенсоры
    for (const auto& sensor : sensors) {
        int x = static_cast<int>(sensor.x * scaleX);
        int y = static_cast<int>(sensor.y * scaleY);
        if (x >= 0 && x < gridX && y >= 0 && y < gridY)
            grid[y][x] = 'S'; // Помечаем сенсор 'S'
    }
    // Добавляем автомобили
    for (const auto& vehicle : trafficFlow) {
        int x = static_cast<int>(vehicle.x * scaleX);
        int y = static_cast<int>(vehicle.y * scaleY);
        if (x >= 0 && x < gridX && y >= 0 && y < gridY)
            grid[y][x] = vehicle.isLeader ? 'L' : 'o';
    }
    // Выводим сетку
    for (const auto& row : grid) {
        for (char c : row) {
            cout << c << " ";
        }
        cout << endl;
    }
}