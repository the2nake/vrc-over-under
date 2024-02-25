/* home.vn2007@gmail.com - 2023 */

#include "main.h"
#include "salsa/auton.hpp"

#include "gui.hpp"

#include <iomanip>
#include <map>
#include <vector>

struct AutonInfo {
  int id = 0;
};

namespace config {
bool program_running;

int program_update_hz;
float joystick_threshold;
double program_delay_per_cycle;

int aps_update_hz;

bool run_auton_before_teleop;
bool init_complete;
int selected_auton;

// button to auton id map here
std::vector<lv_obj_t *> auton_btns;
std::map<lv_obj_t *, AutonInfo> btn_to_info_map;

pros::controller_digital_e_t intake_in;
pros::controller_digital_e_t intake_out;

pros::controller_digital_e_t kicker_shoot;
pros::controller_digital_e_t lift_toggle;
pros::controller_digital_e_t wings_toggle;

}; // namespace config

void lv_tick_loop(void *) {
  while (true) {
    lv_tick_inc(5);
    pros::delay(5);
  }
}

void odom_update_handler(void *params) {
  int update_delay = (int)(1000 / config::aps_update_hz); // in ms
  while (odom != nullptr) {
    if (!sensor_update_paused) {
      imu->update_heading();
      odom->update();
    }

    pros::delay(update_delay);
  }
}

lv_res_t switch_callback(lv_obj_t *sw) {
  config::run_auton_before_teleop = lv_sw_get_state(sw);
  return LV_RES_OK;
}

lv_res_t confirm_callback(lv_obj_t *btn) {
  config::init_complete = true;
  return LV_RES_OK;
}

lv_res_t btn_select_callback(lv_obj_t *selected_btn) {
  if (config::btn_to_info_map.count(selected_btn)) {
    auto data = config::btn_to_info_map.at(selected_btn);
    config::selected_auton = data.id;

    for (auto btn : config::auton_btns) {
      lv_btn_set_state(btn, LV_BTN_STATE_REL);
    }
    lv_btn_set_state(selected_btn, LV_BTN_STATE_TGL_REL);
  }
  return LV_RES_OK;
}

lv_obj_t *new_auton_btn(lv_obj_t *auton_panel, lv_obj_t *prev_btn,
                        AutonInfo data, const char *desc, int w, int h,
                        int margin) {
  lv_obj_t *new_btn = lv_btn_create(auton_panel, nullptr);
  lv_btn_set_toggle(new_btn, true);
  config::btn_to_info_map.insert(
      std::pair<lv_obj_t *, AutonInfo>(new_btn, data));
  config::auton_btns.push_back(new_btn);
  lv_obj_t *new_btn_label = lv_label_create(new_btn, nullptr);
  lv_label_set_text(new_btn_label, desc);
  lv_obj_align(new_btn_label, nullptr, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_size(new_btn, w, h);
  lv_obj_align(new_btn, prev_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, margin);
  lv_btn_set_action(new_btn, LV_BTN_ACTION_CLICK, btn_select_callback);
  return new_btn;
}

void auton_selector() {

  int command_margin = 4;  // px
  int command_height = 32; // px
  int sw_width = command_height * 2;
  int menu_margin = 4; // px
  int menu_width = 240 - 2 * menu_margin;
  int btn_margin = 4; // px
  int auton_btn_w = (int)std::floor(menu_width * 3.0 / 4.0);
  int auton_btn_h = 32;

  // takes up the right half of the screen
  // margins 4px
  // 232 px square

  // scrolling should occur automatically
  /*Create a scroll bar style*/
  static lv_style_t style_sb;
  lv_style_copy(&style_sb, &lv_style_plain);
  style_sb.body.main_color = LV_COLOR_BLACK;
  style_sb.body.grad_color = LV_COLOR_BLACK;
  style_sb.body.border.color = LV_COLOR_WHITE;
  style_sb.body.border.width = 1;
  style_sb.body.border.opa = LV_OPA_70;
  style_sb.body.radius = LV_RADIUS_CIRCLE;
  style_sb.body.opa = LV_OPA_60;
  style_sb.body.padding.hor = 3;   /*Horizontal padding on the right*/
  style_sb.body.padding.inner = 8; /*Scrollbar width*/

  lv_style_btn_ina.body.radius = 4;
  lv_style_btn_tgl_pr.body.radius = 4;
  lv_style_btn_tgl_rel.body.radius = 4;
  lv_style_btn_pr.body.radius = 4;
  lv_style_btn_rel.body.radius = 4;

  lv_obj_t *auton_panel = lv_page_create(lv_scr_act(), nullptr);
  lv_obj_set_size(auton_panel, menu_width,
                  240 - 2 * menu_margin - command_margin - command_height);
  lv_obj_align(auton_panel, nullptr, LV_ALIGN_IN_TOP_RIGHT, -menu_margin,
               menu_margin);
  lv_page_set_style(auton_panel, LV_PAGE_STYLE_SB, &style_sb);
  lv_page_set_sb_mode(auton_panel, LV_SB_MODE_AUTO);

  lv_obj_t *run_before_teleop_switch = lv_sw_create(lv_scr_act(), nullptr);
  lv_sw_off(run_before_teleop_switch);
  lv_obj_set_size(run_before_teleop_switch, sw_width, command_height);
  lv_obj_align(run_before_teleop_switch, nullptr, LV_ALIGN_IN_BOTTOM_LEFT,
               240 + menu_margin, -command_margin);
  lv_sw_set_action(run_before_teleop_switch, switch_callback);

  lv_obj_t *sw_label = lv_label_create(lv_scr_act(), nullptr);
  lv_label_set_text(sw_label, "Run auto?");
  lv_obj_set_width(sw_label, command_height * 2);
  lv_obj_set_height(sw_label, command_height);
  lv_obj_align(sw_label, run_before_teleop_switch, LV_ALIGN_OUT_RIGHT_MID,
               command_margin * 2, 0);

  lv_obj_t *execute_btn = lv_btn_create(lv_scr_act(), nullptr);
  lv_btn_set_action(execute_btn, LV_BTN_ACTION_CLICK, confirm_callback);
  lv_obj_t *execute_btn_label = lv_label_create(execute_btn, nullptr);
  lv_label_set_text(execute_btn_label, "Confirm");
  lv_obj_align(execute_btn_label, nullptr, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_size(execute_btn,
                  240 - 2 * menu_margin - 4 * command_margin -
                      lv_obj_get_width(sw_label) -
                      lv_obj_get_width(run_before_teleop_switch),
                  command_height);
  lv_obj_align(execute_btn, sw_label, LV_ALIGN_OUT_RIGHT_MID,
               2 * command_margin, 0);

  // MENU BUTTONS + DESCRIPTIONS HERE

  // No Auton Button

  lv_obj_t *auton_btn0 = lv_btn_create(auton_panel, nullptr);
  lv_btn_set_toggle(auton_btn0, true);
  lv_btn_set_state(auton_btn0, LV_BTN_STATE_TGL_REL);
  config::btn_to_info_map.insert(
      std::pair<lv_obj_t *, AutonInfo>(auton_btn0, {0}));
  config::auton_btns.push_back(auton_btn0);
  lv_obj_t *auton_btn0_label = lv_label_create(auton_btn0, nullptr);
  lv_label_set_text(auton_btn0_label, "do nothing");
  lv_obj_align(auton_btn0_label, nullptr, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_size(auton_btn0, auton_btn_w, auton_btn_h);
  lv_obj_align(auton_btn0, nullptr, LV_ALIGN_IN_TOP_LEFT, 0, btn_margin);
  lv_btn_set_action(auton_btn0, LV_BTN_ACTION_CLICK, btn_select_callback);

  // Simple AWP Offensive

  auto auton_btn1 = new_auton_btn(auton_panel, auton_btn0, {1}, "awp-o simple",
                                  auton_btn_w, auton_btn_h, btn_margin);

  // 5 Ball AWP Offensive

  auto auton_btn2 = new_auton_btn(auton_panel, auton_btn1, {2}, "awp-o 5ball",
                                  auton_btn_w, auton_btn_h, btn_margin);

  // AWP Safe Defensive

  auto auton_btn3 = new_auton_btn(auton_panel, auton_btn2, {3}, "awp-d safe",
                                  auton_btn_w, auton_btn_h, btn_margin);

  auto auton_btn4 = new_auton_btn(auton_panel, auton_btn3, {4}, "auton skills",
                                  auton_btn_w, auton_btn_h, btn_margin);

  auto auton_btn5 =
      new_auton_btn(auton_panel, auton_btn4, {5}, "auton skills 2", auton_btn_w,
                    auton_btn_h, btn_margin);

  auto auton_btn6 = new_auton_btn(auton_panel, auton_btn5, {6}, "auton driver",
                                  auton_btn_w, auton_btn_h, btn_margin);

  while (!config::init_complete) {
    pros::delay(20);
  }

  /*
    // once confirm is hit, delete everything
    lv_obj_clean(auton_panel);
    lv_obj_clean(run_before_teleop_switch);
    lv_obj_clean(sw_label);
    lv_obj_clean(execute_btn);
    */
}

void initialize() {
  config::init_complete = false;

  // LVGL styles
  lv_style_init();

  pros::Task lvgl_tick{lv_tick_loop};
  lv_init();

  // ===== CONFIGURATION =====

  using namespace config;

  program_update_hz = 40;
  aps_update_hz = 100;

  joystick_threshold = 0.02;

  intake_in = pros::E_CONTROLLER_DIGITAL_R2;
  intake_out = pros::E_CONTROLLER_DIGITAL_R1;
  kicker_shoot = pros::E_CONTROLLER_DIGITAL_L2;
  wings_toggle = pros::E_CONTROLLER_DIGITAL_L1;
  lift_toggle = pros::E_CONTROLLER_DIGITAL_UP;

  selected_auton = 0;

  //  ===== END CONFIG =====

  program_delay_per_cycle =
      std::max(1000.0 / program_update_hz, 5.0); // wait no lower than 5 ms

  initialise_devices();
  initialise_chassis();
  initialise_sensors();

  pros::Task odometry_update{odom_update_handler};

  program_running = true;
  pros::delay(250);

  odom->set_position(0, 0);

  auton_selector();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  auto start = pros::millis();
  uint32_t end = 0;
  // NOTE: 0 heading is straight ahead for the driver
  //       0, 0 is the center of the field

  StarDriveController *drive_controller = StarDriveController::builder()
                                              ->with_drive(chassis)
                                              .with_odometry(odom)
                                              .build();
  drive_controller->configure_pidf_x(1.0 / 100.0, 0.0000004, 8);
  drive_controller->configure_pidf_y(1.0 / 100.0, 0.0000004, 8);
  drive_controller->configure_pidf_r(1.0 / 50.00, 0.0000008, 8);
  drive_controller->configure_stop_threshold(0.06);

  switch (config::selected_auton) {
  case 1:
    // SETUP INFO: goalside route, triball in the intake, facing goal (heading
    // 270), centered on tile
    auton_awp_o_safe(drive_controller, odom);
    break;
  case 2:
    // SETUP INFO: triball on the wedge, wedge aligns with edge of the tile
    // closer to the corner, intake facing hallway triball, centered in the
    // hallway
    auton_awp_o_5_ball(drive_controller, odom);
    break;
  case 3:
    // SETUP INFO: This load side route starts with a triball in the
    // intake, wedge aligned with the edge of the tile near the hang bars, robot
    // aligned with the tile of the hallway (a bit offset from hall center)
    auton_awp_d_safe(drive_controller, odom);
    break;
  case 4:
    // SETUP INFO: start with the robot aligned against the match load zone on
    // the right, intake facing away from the goal. the intake sled should touch
    // the side of the field. one triball on the side of the goal, towards the
    // right, the other in the middle of the wedge.
    auton_skills(drive_controller, odom);
    break;
  case 5:
    // SETUP INFO: start with the robot aligned against the match load zone on
    // the right, intake facing away from the goal. the intake sled should touch
    // the side of the field. one triball on the side of the goal, towards the
    // right, the other in the middle of the wedge.
    auton_skills_2(drive_controller, odom);
    break;
  case 6:
    // SETUP INFO: start with the robot aligned against the match load zone on
    // the right, intake facing away from the goal. the intake sled should touch
    // the side of the field. one triball on the side of the goal, towards the
    // right, the other in the middle of the wedge.
    auton_driver(drive_controller, odom);
    break;
  default:
    odom->set_heading(270);
    break;
  }

  end = pros::millis();
  pros::screen::print(pros::E_TEXT_MEDIUM, 9, "Auton stop (ms): %d",
                      end - start);
}

void intake_control(pros::Controller *controller) {
  if (controller->get_digital(config::intake_in)) {
    motor_intake->move_voltage(12000);
  } else if (controller->get_digital(config::intake_out)) {
    motor_intake->move_voltage(-12000);
  } else {
    motor_intake->brake();
  }
}

void kicker_control(pros::Controller *controller) {
  if (controller->get_digital(config::kicker_shoot)) {
    motor_kicker->move_voltage(12000);
  } else {
    motor_kicker->brake();
  }
}

void wings_control(pros::Controller *controller) {
  if (controller->get_digital_new_press(config::wings_toggle)) {
    toggle_wings();
  }
}

void lift_control(pros::Controller *controller) {
  if (controller->get_digital_new_press(config::lift_toggle)) {
    is_lift_out = !is_lift_out;
    piston_lift->set_value(is_lift_out);
  }
}

void opcontrol() {
  if (config::run_auton_before_teleop) {
    autonomous();
  }

  pros::Controller *controller =
      new pros::Controller(pros::E_CONTROLLER_MASTER);

  // graph setup
  gui::Graph *graph = new gui::Graph();
  graph->set_display_region({244, 4, 232, 232});
  graph->set_window(-1000.0, -1000.0, 2000.0, 2000.0);
  graph->point_width = 3;
  std::vector<Point<double>> points = {{2.0, 2.0}};

  while (config::program_running) {
    auto cycle_start = pros::millis();

    // INPUT
    auto input_lx = controller->get_analog(ANALOG_LEFT_X) / 127.0;
    auto input_rx = controller->get_analog(ANALOG_RIGHT_X) / 127.0;
    auto input_ry = controller->get_analog(ANALOG_RIGHT_Y) / 127.0;

    // OUTPUT
    if (std::abs(input_lx) < config::joystick_threshold &&
        std::abs(input_rx) < config::joystick_threshold &&
        std::abs(input_ry) < config::joystick_threshold) {
      chassis->brake();
    } else {
      auto velocities = chassis->drive_field_based(input_rx, input_ry, input_lx,
                                                   odom->get_pose().heading);
    }

    if (controller->get_digital_new_press(DIGITAL_A)) {
      odom->set_heading(0);
    }

    intake_control(controller);
    wings_control(controller);
    lift_control(controller);
    kicker_control(controller);

    // debug
    Pose pose = odom->get_pose();
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X, Y: %.2f, %.2f", pose.x,
                        pose.y);
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Heading: %.2f", pose.heading);

    if (points.size() > 100) {
      points.erase(points.begin());
    }
    points.push_back({pose.x, pose.y});

    graph->draw();
    graph->plot(points);

    double cycle_time = pros::millis() - cycle_start;
    pros::delay(std::max(0.0, config::program_delay_per_cycle - cycle_time));
  }
}
