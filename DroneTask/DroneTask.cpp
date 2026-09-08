#include <iostream>
#include <windows.h>
#include <chrono>
#include <thread>
#include <vector>
#include <fstream>
#include <memory>
#include <cstdlib>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

using namespace std;
using namespace std::chrono;
using namespace mavsdk;

// Клас, що реалізує класичний ПІД-регулятор (Пропорційно-Інтегрально-Диференціальний)
// Використовується для розрахунку необхідної тяги моторів на основі похибки висоти
class PIDController {
public:
    // Конструктор приймає коефіцієнти регулятора, базову тягу висіння та крок часу
    PIDController(double kp, double ki, double kd, double hover_thrust, double dt)
        : kp_(kp), ki_(ki), kd_(kd), hover_thrust_(hover_thrust), dt_(dt), integral_(0.0), prev_error_(0.0) {
    }

    // Головний метод обчислення керуючого впливу (тяги) на поточному кроці
    double calculate(double setpoint, double current_value) {
        // Обчислення поточної похибки (різниці між бажаною та реальною висотою)
        double error = setpoint - current_value;

        // Інтегральна складова: накопичує похибку з часом для усунення статичної помилки
        integral_ += error * dt_;

        // Anti-windup захист: обмеження накопичення інтегралу, щоб уникнути різких стрибків
        if (integral_ > 2.0) integral_ = 2.0;
        if (integral_ < -2.0) integral_ = -2.0;

        // Пропорційна складова: безпосередня реакція на поточну похибку
        double p = kp_ * error;
        // Інтегральна складова: реакція на сумарну історичну похибку
        double i = ki_ * integral_;
        // Диференціальна складова: реакція на швидкість зміни похибки (гальмування)
        double d = kd_ * (error - prev_error_) / dt_;

        // Збереження поточної похибки для наступного кроку (розрахунку похідної)
        prev_error_ = error;

        // Сумарна тяга = базова тяга зависання + коригування ПІД-регулятора
        double thrust = hover_thrust_ + p + i + d;

        // Обмеження вихідного сигналу тяги в межах апаратних можливостей дрона [0.0; 1.0]
        if (thrust > 1.0) thrust = 1.0;
        if (thrust < 0.0) thrust = 0.0;

        return thrust;
    }

private:
    double kp_, ki_, kd_, hover_thrust_, dt_;
    double integral_, prev_error_;
};

// Клас для управління симулятором квадрокоптера через протокол MAVLink
class CopterSITL {
public:
    // Ініціалізація плагінів MAVSDK для отримання телеметрії та відправки низькорівневих команд
    CopterSITL(shared_ptr<mavsdk::System> sys) : system_(sys) {
        telemetry_ = make_shared<Telemetry>(system_);
        param_ = make_shared<Param>(system_);
        passthrough_ = make_shared<MavlinkPassthrough>(system_);

        // Встановлення частоти оновлення даних позиції для більшої точності керування (10 Гц)
        telemetry_->set_rate_position(10.0);
    }

    // Налаштування параметрів поведінки режиму GUIDED в ArduPilot
    void set_guid_options() {
        // GUID_OPTIONS=8 дозволяє системі приймати сирі команди керування положенням/тягою (Attitude Target)
        param_->set_param_int("GUID_OPTIONS", 8);
        cout << "Параметр GUID_OPTIONS встановлено в 8" << endl;
    }

    // Відправка низькорівневої команди MAVLink для зміни польотного режиму
    void set_mode(int custom_mode) {
        mavlink_message_t msg;
        // Пакування команди MAV_CMD_DO_SET_MODE. custom_mode 4 = GUIDED, 9 = LAND
        mavlink_msg_command_long_pack(
            passthrough_->get_our_sysid(), passthrough_->get_our_compid(), &msg,
            system_->get_system_id(), 1,
            MAV_CMD_DO_SET_MODE, 0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode, 0, 0, 0, 0, 0
        );
        passthrough_->send_message(msg);
    }

    // Розблокування моторів дрона (Arming) перед зльотом
    void arm() {
        mavlink_message_t msg;
        mavlink_msg_command_long_pack(
            passthrough_->get_our_sysid(), passthrough_->get_our_compid(), &msg,
            system_->get_system_id(), 1,
            MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        );
        passthrough_->send_message(msg);
        cout << "Дрон заармлено!" << endl;
    }

    // Відправка розрахованого значення тяги на польотний контролер
    void send_thrust(float thrust) {
        mavlink_message_t msg;
        // Кватерніон, що задає нейтральне положення дрона (без нахилів: roll=0, pitch=0)
        float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
        float thrust_body[3] = { 0.0f, 0.0f, 0.0f };

        // Відправка команди MAVLINK_MSG_ID_SET_ATTITUDE_TARGET. 
        // Маска 135 ігнорує команди швидкостей обертання і використовує лише кватерніон та вектор тяги
        mavlink_msg_set_attitude_target_pack(
            passthrough_->get_our_sysid(), passthrough_->get_our_compid(), &msg, 0,
            system_->get_system_id(), 1,
            135, q, 0.0f, 0.0f, 0.0f, thrust, thrust_body
        );
        passthrough_->send_message(msg);
    }

    // Отримання поточної відносної висоти дрона з телеметрії (в метрах)
    float get_altitude() { return telemetry_->position().relative_altitude_m; }

    // Головний цикл керування висотою на заданий проміжок часу
    void fly_to_altitude(float target_alt, double duration, double dt = 0.1) {
        cout << "Прямуємо до висоти " << target_alt << " метрів..." << endl;
        // Ініціалізація ПІД-регулятора з відкаліброваними коефіцієнтами
        PIDController pid(0.18, 0.06, 0.2, 0.5, dt);

        auto end_time = steady_clock::now() + milliseconds(static_cast<int>(duration * 1000));
        auto start_loop_time = steady_clock::now();

        // Цикл виконання діє до закінчення заданого часу duration
        while (steady_clock::now() < end_time) {
            auto iter_start = steady_clock::now();

            // Зчитування поточної висоти
            float current_alt = get_altitude();
            // Розрахунок необхідної тяги та відправка її на дрон
            send_thrust(static_cast<float>(pid.calculate(target_alt, current_alt)));

            // Логування даних (час, реальна висота, цільова висота) для побудови графіка
            double elapsed = duration_cast<milliseconds>(steady_clock::now() - start_loop_time).count() / 1000.0;
            time_log_.push_back(elapsed + global_time_offset_);
            alt_log_.push_back(current_alt);
            setpoint_log_.push_back(target_alt);

            // Синхронізація частоти циклу з налаштованим dt (0.1 сек = 10 Гц)
            auto iter_end = steady_clock::now();
            int sleep_time = static_cast<int>(dt * 1000) - duration_cast<milliseconds>(iter_end - iter_start).count();
            if (sleep_time > 0) this_thread::sleep_for(milliseconds(sleep_time));
        }
        // Збереження глобального часу для коректного продовження логування в наступних фазах
        global_time_offset_ += duration;
    }

    // Експорт зібраних логів польоту у формат CSV для подальшого аналізу
    void save_csv() {
        ofstream file("flight_log.csv");
        file << "Time,Altitude,Setpoint\n";
        for (size_t i = 0; i < time_log_.size(); ++i) {
            file << time_log_[i] << "," << alt_log_[i] << "," << setpoint_log_[i] << "\n";
        }
    }

private:
    shared_ptr<mavsdk::System> system_;
    shared_ptr<Telemetry> telemetry_;
    shared_ptr<Param> param_;
    shared_ptr<MavlinkPassthrough> passthrough_;
    vector<double> time_log_, alt_log_, setpoint_log_;
    double global_time_offset_ = 0.0;
};

int main() {
    // Встановлення кодової сторінки 1251 для коректного відображення кирилиці в консолі Windows
    SetConsoleCP(1251);
    SetConsoleOutputCP(1251);

    // Ініціалізація ядра MAVSDK як наземної станції (Ground Station)
    Mavsdk::Configuration config{ ComponentType::GroundStation };
    Mavsdk mavsdk{ config };

    // Спроба підключення до симулятора SITL через TCP
    if (mavsdk.add_any_connection("tcp://127.0.0.1:5762") != ConnectionResult::Success) {
        cerr << "Помилка підключення" << endl; return 1;
    }

    cout << "Очікування підключення дрона..." << endl;

    // Пошук автопілота в мережі (таймаут 3 секунди)
    auto system = mavsdk.first_autopilot(3.0);
    if (!system) { cerr << "Дрон не знайдено." << endl; return 1; }

    CopterSITL drone(system.value());

    // Налаштування автопілота для прийому зовнішніх команд тяги
    drone.set_guid_options();
    this_thread::sleep_for(seconds(1));

    // Переключення в режим GUIDED (режим зовнішнього керування)
    drone.set_mode(4); // 4 відповідає режиму GUIDED в ArduCopter
    this_thread::sleep_for(seconds(1));

    // Розблокування моторів
    drone.arm();
    this_thread::sleep_for(seconds(2)); // Пауза для розкрутки моторів

    // Фаза 1: Зліт та утримання висоти 10 метрів протягом 20 секунд
    drone.fly_to_altitude(10.0, 20.0);
    // Фаза 2: Зниження та утримання висоти 5 метрів протягом 15 секунд
    drone.fly_to_altitude(5.0, 15.0);

    // Переключення в режим посадки
    drone.set_mode(9); // 9 відповідає режиму LAND в ArduCopter
    cout << "Режим LAND активовано." << endl;

    // Збереження даних телеметрії та виклик Python-скрипта для візуалізації
    drone.save_csv();
    std::system("python plot_graph.py");

    return 0;
}