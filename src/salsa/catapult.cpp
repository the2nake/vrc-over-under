#include "salsa/catapult.hpp"
#include "salsa/api.hpp"

pros::Mutex cata_state_mutex;
std::atomic<bool> firing = false;
std::atomic<bool> loading = false;

void initialise_catapult() {
  load_catapult();
}

bool catapult_is_loaded() { return catapult_loaded_switch->get_value(); }

void load_catapult(void *params) {
  if (pto.load() == true) {
    // wait until no longer firing
    while (firing.load() == true) {
      pros::delay(20);
    }

    // notify other tasks that loading is in progress
    while (!cata_state_mutex.take(5)) {
      pros::delay(1);
    }
    loading = true;
    cata_state_mutex.give();

    while (!catapult_is_loaded()) {
      motor_pto_l->move_voltage(12000);
      motor_pto_r->move_voltage(12000);
      pros::delay(20);
    }

    // loading finished, reset the flag
    while (!cata_state_mutex.take(5)) {
      pros::delay(1);
    }
    loading = false;
    cata_state_mutex.give();

    // brake the motors
    motor_pto_l->brake();
    motor_pto_r->brake();
  }
}

void load_catapult_async() {
  // create a task to run the code in the background
  pros::Task catapult_load_task{load_catapult, nullptr, "catapult load task"};
}

void fire_catapult(void *params) {
  if (pto.load() == true) {
    // wait for loading to finish
    while (loading.load() == true) {
      pros::delay(20);
    }

    // notify other tasks that firing is in progress
    while (!cata_state_mutex.take(5)) {
      pros::delay(1);
    }
    firing = true;
    cata_state_mutex.give();

    while (catapult_is_loaded()) {
      motor_pto_l->move_voltage(12000);
      motor_pto_r->move_voltage(12000);
      pros::delay(20);
    }

    // firing finished, reset the flag
    while (!cata_state_mutex.take(5)) {
      pros::delay(1);
    }
    firing = false;
    cata_state_mutex.give();

    // brake the motors
    motor_pto_l->brake();
    motor_pto_r->brake();
  }
}

void fire_catapult_async() {
  pros::Task catapult_fire_task{fire_catapult, nullptr, "catapult fire task"};
}
