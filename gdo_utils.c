/* GdoLib - A library for controlling garage door openers.
 * Copyright (C) 2024  Konnected Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "gdo.h"
#include "gdo_priv.h"

const char *gdo_door_state_str[] = {
    "Unknown",
    "Open",
    "Closed",
    "Stopped",
    "Opening",
    "Closing",
};

const char *gdo_light_state_str[] = {
    "Off",
    "On",
    "Unknown",
};

const char *gdo_lock_state_str[] = {
    "Unlocked",
    "Locked",
    "Unknown",
};

const char *gdo_motion_state_str[] = {
    "Clear",
    "Detected",
    "Unknown",
};

const char *gdo_obstruction_state_str[] = {
    "Obstructed",
    "Clear",
    "Unknown",
};

const char *gdo_motor_state_str[] = {
    "Off",
    "On",
    "Unknown",
};

const char *gdo_button_state_str[] = {
    "Pressed",
    "Released",
    "Unknown",
};

const char *gdo_battery_state_str[] = {
    "Unknown",
    "Charging",
    "Full",
};

const char *gdo_learn_state_str[] = {
    "Inactive",
    "Active",
    "Unknown",
};

const char *gdo_paired_device_type_str[] = {
    "All",
    "Remote",
    "Keypad",
    "Wall Control",
    "Accessory",
    "Unknown",
};

const char *gdo_light_action_str[] = {
    "Off",
    "On",
    "Toggle",
    "Unknown",
};

const char *gdo_lock_action_str[] = {
    "Unlock",
    "Lock",
    "Toggle",
    "Unknown",
};

const char *gdo_door_action_str[] = {
    "Close",
    "Open",
    "Toggle",
    "Stop",
    "Unknown",
};

const char *gdo_protocol_type_str[] = {
    "Unknown",
    "Security+ 1.0",
    "Security+ 2.0",
};

const char* cmd_to_string(gdo_command_t cmd) {
    switch (cmd) {
        case GDO_CMD_GET_STATUS:
            return "GET_STATUS";
        case GDO_CMD_STATUS:
            return "STATUS";
        case GDO_CMD_OBST_1:
            return "OBST_1";
        case GDO_CMD_OBST_2:
            return "OBST_2";
        case GDO_CMD_BATTERY_STATUS:
            return "BATTERY_STATUS";
        case GDO_CMD_PAIR_3:
            return "PAIR_3";
        case GDO_CMD_PAIR_3_RESP:
            return "PAIR_3_RESP";
        case GDO_CMD_LEARN:
            return "LEARN";
        case GDO_CMD_LOCK:
            return "LOCK";
        case GDO_CMD_DOOR_ACTION:
            return "DOOR_ACTION";
        case GDO_CMD_LIGHT:
            return "LIGHT";
        case GDO_CMD_MOTOR_ON:
            return "MOTOR_ON";
        case GDO_CMD_MOTION:
            return "MOTION";
        case GDO_CMD_GET_PAIRED_DEVICES:
            return "GET_PAIRED_DEVICES";
        case GDO_CMD_PAIRED_DEVICES:
            return "PAIRED_DEVICES";
        case GDO_CMD_CLEAR_PAIRED_DEVICES:
            return "CLEAR_PAIRED_DEVICES";
        case GDO_CMD_LEARN_1:
            return "LEARN_1";
        case GDO_CMD_PING:
            return "PING";
        case GDO_CMD_PING_RESP:
            return "PING_RESP";
        case GDO_CMD_PAIR_2:
            return "PAIR_2";
        case GDO_CMD_PAIR_2_RESP:
            return "PAIR_2_RESP";
        case GDO_CMD_SET_TTC:
            return "SET_TTC";
        case GDO_CMD_CANCEL_TTC:
            return "CANCEL_TTC";
        case GDO_CMD_TTC:
            return "TTC";
        case GDO_CMD_GET_OPENINGS:
            return "GET_OPENINGS";
        case GDO_CMD_OPENINGS:
            return "OPENINGS";
        default:
            return "UNKNOWN";
    }
}

const char* v1_cmd_to_string(gdo_v1_command_t cmd) {
    switch (cmd) {
        case V1_CMD_TOGGLE_DOOR_PRESS:
            return "TOGGLE_DOOR_PRESS";
        case V1_CMD_TOGGLE_DOOR_RELEASE:
            return "TOGGLE_DOOR_RELEASE";
        case V1_CMD_TOGGLE_LIGHT_PRESS:
            return "TOGGLE_LIGHT_PRESS";
        case V1_CMD_TOGGLE_LIGHT_RELEASE:
            return "TOGGLE_LIGHT_RELEASE";
        case V1_CMD_TOGGLE_LOCK_PRESS:
            return "TOGGLE_LOCK_PRESS";
        case V1_CMD_TOGGLE_LOCK_RELEASE:
            return "TOGGLE_LOCK_RELEASE";
        case V1_CMD_QUERY_DOOR_STATUS_0x37:
            return "QUERY_DOOR_STATUS_0x37";
        case V1_CMD_QUERY_DOOR_STATUS:
            return "QUERY_DOOR_STATUS";
        case V1_CMD_OBSTRUCTION:
            return "OBSTRUCTION";
        case V1_CMD_QUERY_OTHER_STATUS:
            return "QUERY_OTHER_STATUS";
        default:
            return "UNKNOWN";
    }
}

void print_buffer(gdo_protocol_type_t protocol, uint8_t* buf, bool is_tx) {
    if (protocol == GDO_PROTOCOL_SEC_PLUS_V2) {
        ESP_LOGD(TAG, "%s: "
                 "[%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X]",
                 is_tx ? "TX" : "RX",
                 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9],
                 buf[10], buf[11], buf[12],buf[13], buf[14], buf[15], buf[16], buf[17], buf[18]);
    } else if (is_tx) {
        ESP_LOGD(TAG, "TX [%02X]", buf[0]);
    } else {
        ESP_LOGD(TAG, "RX [%02X %02X]", buf[0], buf[1]);
    }
}

const char* gdo_door_state_to_string(gdo_door_state_t state) {
    return gdo_door_state_str[state];
}

const char* gdo_light_state_to_string(gdo_light_state_t state) {
    return gdo_light_state_str[state];
}

const char* gdo_lock_state_to_string(gdo_lock_state_t state) {
    return gdo_lock_state_str[state];
}

const char* gdo_motion_state_to_string(gdo_motion_state_t state) {
    return gdo_motion_state_str[state];
}

const char* gdo_obstruction_state_to_string(gdo_obstruction_state_t state) {
    return gdo_obstruction_state_str[state];
}

const char* gdo_motor_state_to_string(gdo_motor_state_t state) {
    return gdo_motor_state_str[state];
}

const char* gdo_button_state_to_string(gdo_button_state_t state) {
    return gdo_button_state_str[state];
}

const char* gdo_battery_state_to_string(gdo_battery_state_t state) {
    return gdo_battery_state_str[state];
}

const char* gdo_learn_state_to_string(gdo_learn_state_t state) {
    return gdo_learn_state_str[state];
}

const char* gdo_paired_device_type_to_string(gdo_paired_device_type_t type) {
    return gdo_paired_device_type_str[type];
}

const char* gdo_light_action_to_string(gdo_light_action_t action) {
    return gdo_light_action_str[action];
}

const char* gdo_lock_action_to_string(gdo_lock_action_t action) {
    return gdo_lock_action_str[action];
}

const char* gdo_door_action_to_string(gdo_door_action_t action) {
    return gdo_door_action_str[action];
}

const char* gdo_protocol_type_to_string(gdo_protocol_type_t protocol) {
    return gdo_protocol_type_str[protocol];
}
