#include "buzzer/engine.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
  try {
    buzzer::HardwareBuzzerConfig cfg;
    cfg.simulate = true;

    if (argc > 1) {
      cfg.simulate = false;
    }

    buzzer::BuzzerEngine engine(cfg);
    engine.begin();

    if (argc > 1) {
      engine.load_profile_json(argv[1]);
    }

    std::cout << "Triggering READY_TO_ARM event...\n";
    engine.notify(buzzer::EventId::ReadyToArm);
    while (engine.is_playing()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Triggering LOST_VEHICLE alarm for 3 seconds...\n";
    engine.set_alarm(buzzer::AlarmId::LostVehicle, true);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    engine.set_alarm(buzzer::AlarmId::LostVehicle, false);

    engine.close();
    return EXIT_SUCCESS;
  } catch (const std::exception& ex) {
    std::cerr << "buzzer example failed: " << ex.what() << "\n";
    return EXIT_FAILURE;
  }
}
