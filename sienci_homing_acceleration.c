/*
  sienci_homing_acceleration.c - per-axis homing acceleration overrides

  Part of grblHAL
*/

#include "driver.h"

#if SIENCI_HOMING_ACCEL_ENABLE

#include "grbl/nvs_buffer.h"

typedef struct {
    coord_data_t acceleration;
} plugin_settings_t;

static nvs_address_t nvs_address;
static plugin_settings_t homing_acceleration;
static on_report_options_ptr on_report_options;
static on_homing_rate_set_ptr on_homing_rate_set;
static on_homing_completed_ptr on_homing_completed;
static axes_signals_t overridden = {0};

static inline void restore_axis_acceleration (uint_fast8_t axis)
{
    if(overridden.mask & bit(axis)) {
        settings_override_acceleration(axis, 0.0f);
        overridden.mask &= ~bit(axis);
    }
}

static inline void apply_axis_acceleration (uint_fast8_t axis)
{
    float acceleration = homing_acceleration.acceleration.values[axis];

    if(acceleration > 0.0f && settings_override_acceleration(axis, acceleration))
        overridden.mask |= bit(axis);
    else
        restore_axis_acceleration(axis);
}

static void update_homing_acceleration (axes_signals_t axes)
{
    uint_fast8_t axis = 0;
    axes_signals_t restore = { .mask = overridden.mask & ~axes.mask };

    while(restore.mask) {
        if(restore.mask & 0x1)
            restore_axis_acceleration(axis);
        restore.mask >>= 1;
        axis++;
    }

    for(axis = 0; axis < N_AXIS; axis++) {
        if(axes.mask & bit(axis))
            apply_axis_acceleration(axis);
    }
}

static status_code_t set_axis_setting (setting_id_t setting, float value)
{
    uint_fast8_t idx;
    status_code_t status = Status_OK;

    switch(settings_get_axis_base(setting, &idx)) {

        case Setting_AxisExtended8:
            homing_acceleration.acceleration.values[idx] = value;
            break;

        default:
            status = Status_SettingDisabled;
            break;
    }

    return status;
}

static float get_float (setting_id_t setting)
{
    uint_fast8_t idx;
    float value = 0.0f;

    switch(settings_get_axis_base(setting, &idx)) {

        case Setting_AxisExtended8:
            value = homing_acceleration.acceleration.values[idx];
            break;

        default:
            break;
    }

    return value;
}

PROGMEM static const setting_detail_t plugin_settings[] = {
    { Setting_AxisExtended8, Group_Axis0, "-axis homing acceleration", "mm/sec^2", Format_Decimal, "#####0.000", "0", NULL, Setting_IsLegacyFn, set_axis_setting, get_float, NULL, { .subgroups = On, .increment = 1 } }
};

PROGMEM static const setting_descr_t plugin_settings_descr[] = {
    { Setting_AxisExtended8, "Acceleration override used while the axis is actively homing. Set to 0 to use the normal axis acceleration." }
};

static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&homing_acceleration, sizeof(plugin_settings_t), true);
}

static void plugin_settings_restore (void)
{
    uint_fast8_t axis = N_AXIS;

    do {
        axis--;
        homing_acceleration.acceleration.values[axis] = settings.axis[axis].acceleration / (60.0f * 60.0f);
    } while(axis);

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&homing_acceleration, sizeof(plugin_settings_t), true);
}

static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&homing_acceleration, nvs_address, sizeof(plugin_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();
}

static void onHomingRateSet (axes_signals_t axes, coord_data_t *feedrate, homing_mode_t mode)
{
    update_homing_acceleration(axes);

    if(on_homing_rate_set)
        on_homing_rate_set(axes, feedrate, mode);
}

static void onHomingCompleted (axes_signals_t cycle, bool success)
{
    uint_fast8_t axis;

    for(axis = 0; axis < N_AXIS; axis++) {
        if(cycle.mask & bit(axis))
            restore_axis_acceleration(axis);
    }

    if(on_homing_completed)
        on_homing_completed(cycle, success);
}

static void on_report_my_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Sienci Homing Acceleration Plugin", "0.01");
}

void sienci_homing_acceleration_plugin_init (void)
{
    static setting_details_t setting_details = {
        .settings = plugin_settings,
        .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
        .descriptions = plugin_settings_descr,
        .n_descriptions = sizeof(plugin_settings_descr) / sizeof(setting_descr_t),
        .save = plugin_settings_save,
        .load = plugin_settings_load,
        .restore = plugin_settings_restore
    };

    if((nvs_address = nvs_alloc(sizeof(plugin_settings_t)))) {
        on_homing_rate_set = grbl.on_homing_rate_set;
        grbl.on_homing_rate_set = onHomingRateSet;

        on_homing_completed = grbl.on_homing_completed;
        grbl.on_homing_completed = onHomingCompleted;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = on_report_my_options;

        settings_register(&setting_details);
    }
}

#endif // SIENCI_HOMING_ACCEL_ENABLE
