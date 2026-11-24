// input_processor_accel_main.c - ZMK Input Processor for Pointing Device Acceleration
// Main event handler and device registration
// 
// Copyright (c) 2024 The ZMK Contributors
// Modifications (c) 2025 NUOVOTAKA
// SPDX-License-Identifier: MIT

#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/input_processor.h>
#include "../include/drivers/input_processor_accel.h"
#include "config/accel_config.h"
#include "config/accel_device_init.h"

LOG_MODULE_REGISTER(input_processor_accel, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_input_processor_acceleration

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

// =============================================================================
// MEMORY POOL DEFINITION
// =============================================================================

// Define the memory pool for acceleration data (only once in main.c)
#if defined(ACCEL_HAS_SMOOTH)
#define ACCEL_SLAB_ALIGNMENT __alignof__(struct accel_data)
#else
#define ACCEL_SLAB_ALIGNMENT ACCEL_DATA_POOL_ALIGNMENT
#endif
K_MEM_SLAB_DEFINE(accel_data_pool, sizeof(struct accel_data), ACCEL_MAX_INSTANCES, ACCEL_SLAB_ALIGNMENT);

// =============================================================================
// DEVICE INITIALIZATION
// =============================================================================

static int accel_init_device(const struct device *dev) {
    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;

    // Validate final configuration
    int ret = accel_validate_config(cfg);
    if (ret < 0) {
        LOG_ERR("Device %s: Configuration validation failed: %d", dev->name, ret);
        return ret; // Pass through validation error code
    }
    
    // Enhanced NULL pointer validation and initialization
    if (!data) {
        LOG_ERR("Device %s: Data structure is NULL", dev->name ? dev->name : "unknown");
        return ACCEL_ERR_NO_MEMORY;
    }
    
    // Initialize runtime data structures - ensure proper memory initialization
    memset(data, 0, sizeof(struct accel_data));
    // Initialize timing data to prevent division by zero
    data->last_time_ms = k_uptime_get_32();
    data->recent_speed = 0;
    
    LOG_INF("Device %s: Acceleration processor ready (Level %d)", dev->name, cfg->level);
    return 0;
}

// =============================================================================
// DEVICE INSTANCE CREATION USING DT_INST_FOREACH_STATUS_OKAY
// =============================================================================

#define SMOOTH_DT_ASSERT(inst)                                                                    \
    BUILD_ASSERT(IN_RANGE(DT_INST_PROP(inst, curve), ACCEL_CURVE_STANDARD, ACCEL_CURVE_SMOOTH));   \
    BUILD_ASSERT(IN_RANGE(DT_INST_PROP(inst, low_gain), 200, 2000));                               \
    BUILD_ASSERT(IN_RANGE(DT_INST_PROP(inst, high_gain), 1000, 6000));                             \
    BUILD_ASSERT(IN_RANGE(DT_INST_PROP(inst, mid_speed), 1, 100));                                 \
    BUILD_ASSERT(IN_RANGE(DT_INST_PROP(inst, ramp_width), 1, 100));                               \
    BUILD_ASSERT(IN_RANGE(DT_INST_PROP(inst, smoothing), 1, 16));                                  \
    BUILD_ASSERT(DT_INST_PROP(inst, high_gain) >= DT_INST_PROP(inst, low_gain));                   \
    BUILD_ASSERT(DT_INST_PROP(inst, ramp_width) <= DT_INST_PROP(inst, mid_speed));                 \
    BUILD_ASSERT(DT_INST_PROP(inst, curve) != ACCEL_CURVE_SMOOTH ||                               \
                 IS_ENABLED(ACCEL_HAS_SMOOTH));

#if defined(ACCEL_HAS_RUNTIME_PARAMS)
#define ACCEL_INIT_CURVE(inst, cfg)                                                               \
    if ((cfg)->level == 2) {                                                                      \
        (cfg)->cfg.level2.curve = DT_INST_PROP(inst, curve);                                       \
        (cfg)->cfg.level2.low_gain = DT_INST_PROP(inst, low_gain);                                 \
        (cfg)->cfg.level2.high_gain = DT_INST_PROP(inst, high_gain);                               \
        (cfg)->cfg.level2.mid_speed = DT_INST_PROP(inst, mid_speed);                               \
        (cfg)->cfg.level2.ramp_width = DT_INST_PROP(inst, ramp_width);                            \
        (cfg)->cfg.level2.smoothing = DT_INST_PROP(inst, smoothing);                             \
    }
#else
#define ACCEL_INIT_CURVE(inst, cfg)
#endif

// Macro to create device instance initialization function
#define ACCEL_INIT_FUNC(inst)                                                                     \
    static int accel_init_##inst(const struct device *dev) {                                     \
        /* Initialize device instance configuration */                                           \
        int ret = accel_device_init_instance(dev, inst);                                        \
        if (ret < 0) {                                                                           \
            return ret;                                                                          \
        }                                                                                        \
                                                                                                  \
        /* Apply DT custom properties if enabled (must be done in macro context) */            \
        struct accel_config *cfg = (struct accel_config *)dev->config;                          \
        bool use_custom_config = IS_ENABLED(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_CUSTOM);       \
        if (use_custom_config) {                                                                  \
            /* Apply common DTS properties for both levels */                                   \
            if (cfg->level == 1) { \
                cfg->cfg.level1.sensitivity = ACCEL_CLAMP(DT_INST_PROP_OR(inst, sensitivity, cfg->cfg.level1.sensitivity), SENSITIVITY_MIN, SENSITIVITY_MAX); \
                cfg->cfg.level1.max_factor = ACCEL_CLAMP(DT_INST_PROP_OR(inst, max_factor, cfg->cfg.level1.max_factor), MAX_FACTOR_MIN, MAX_FACTOR_MAX); \
                cfg->cfg.level1.curve_type = ACCEL_CLAMP(DT_INST_PROP_OR(inst, curve_type, cfg->cfg.level1.curve_type), CURVE_TYPE_MIN, CURVE_TYPE_MAX); \
            } else { \
                cfg->cfg.level2.sensitivity = ACCEL_CLAMP(DT_INST_PROP_OR(inst, sensitivity, cfg->cfg.level2.sensitivity), SENSITIVITY_MIN, SENSITIVITY_MAX); \
                cfg->cfg.level2.max_factor = ACCEL_CLAMP(DT_INST_PROP_OR(inst, max_factor, cfg->cfg.level2.max_factor), MAX_FACTOR_MIN, MAX_FACTOR_MAX); \
            } \
            cfg->y_boost_scaled = ACCEL_CLAMP((DT_INST_PROP_OR(inst, y_boost, SENSITIVITY_SCALE) - SENSITIVITY_SCALE) / 10, 0, 200); \
            uint16_t dpi = ACCEL_CLAMP(DT_INST_PROP_OR(inst, sensor_dpi, STANDARD_DPI_REFERENCE), SENSOR_DPI_MIN, SENSOR_DPI_MAX); \
            cfg->sensor_dpi_class = (dpi <= 400) ? 0 : (dpi <= 800) ? 1 : (dpi <= 1200) ? 2 : (dpi <= 1600) ? 3 : (dpi <= 3200) ? 4 : (dpi <= 6400) ? 5 : 6; \
                                                                                                  \
            /* Apply Level 2 specific DTS properties only for Standard level */                \
            if (cfg->level == 2) {                                                              \
                cfg->cfg.level2.speed_threshold = ACCEL_CLAMP(DT_INST_PROP_OR(inst, speed_threshold, cfg->cfg.level2.speed_threshold), SPEED_THRESHOLD_MIN, SPEED_THRESHOLD_MAX); \
                cfg->cfg.level2.speed_max = ACCEL_CLAMP(DT_INST_PROP_OR(inst, speed_max, cfg->cfg.level2.speed_max), SPEED_MAX_MIN, SPEED_MAX_MAX); \
                cfg->cfg.level2.min_factor = ACCEL_CLAMP(DT_INST_PROP_OR(inst, min_factor, cfg->cfg.level2.min_factor), MIN_FACTOR_MIN, MIN_FACTOR_MAX); \
                cfg->cfg.level2.acceleration_exponent = ACCEL_CLAMP(DT_INST_PROP_OR(inst, acceleration_exponent, cfg->cfg.level2.acceleration_exponent), ACCEL_EXPONENT_MIN, ACCEL_EXPONENT_MAX); \
            }                                                                                    \
        }                                                                                        \
                                                                                                  \
        ACCEL_INIT_CURVE(inst, cfg)                                                               \
        /* Final device initialization and validation */                                        \
        return accel_init_device(dev);                                                          \
    }

// Macro to create device instance data and config structures
#define ACCEL_DEVICE_DEFINE(inst)                                                               \
    SMOOTH_DT_ASSERT(inst)                                                                      \
    static struct accel_data accel_data_##inst = {0};                                          \
    static struct accel_config accel_config_##inst = {0};                                      \
    ACCEL_INIT_FUNC(inst)                                                                      \
    DEVICE_DT_INST_DEFINE(inst,                                                                \
                          accel_init_##inst,                                                    \
                          NULL,                                                                 \
                          &accel_data_##inst,                                                   \
                          &accel_config_##inst,                                                 \
                          POST_KERNEL,                                                          \
                          CONFIG_INPUT_PROCESSOR_ACCELERATION_INIT_PRIORITY,                    \
                          &(const struct zmk_input_processor_driver_api){                      \
                              .handle_event = accel_handle_event                                \
                          });

// Create device instances for all enabled DT nodes
DT_INST_FOREACH_STATUS_OKAY(ACCEL_DEVICE_DEFINE)

#if !defined(CONFIG_ZMK_SPLIT) || defined(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static struct k_spinlock config_lock;

int accel_get_params(const struct device *dev, struct accel_params *out) {
    if (!dev || !out || !device_is_ready(dev)) {
        return -EINVAL;
    }
    const struct accel_config *cfg = dev->config;
    if (cfg->level != 2) {
        return -EINVAL;
    }
    k_spinlock_key_t key = k_spin_lock(&config_lock);
    *out = (struct accel_params){
        .sensitivity = cfg->cfg.level2.sensitivity,
        .min_factor = cfg->cfg.level2.min_factor,
        .max_factor = cfg->cfg.level2.max_factor,
        .speed_threshold = cfg->cfg.level2.speed_threshold,
        .speed_max = cfg->cfg.level2.speed_max,
        .y_boost = accel_decode_y_boost(cfg->y_boost_scaled),
        .curve = cfg->cfg.level2.curve,
#define COPY(field, lo, hi) .field = cfg->cfg.level2.field,
        ACCEL_SMOOTH_FIELDS(COPY)
#undef COPY
    };
    k_spin_unlock(&config_lock, key);
    return 0;
}

int accel_set_params(const struct device *dev, const struct accel_params *in) {
    if (!dev || !in || !device_is_ready(dev)) {
        return -EINVAL;
    }
    struct accel_config *cfg = (struct accel_config *)dev->config;
    if (cfg->level != 2) {
        return -EINVAL;
    }
#define INVALID(field, lo, hi) !IN_RANGE(in->field, lo, hi) ||
    if (!IN_RANGE(in->sensitivity, SENSITIVITY_MIN, 1000) ||
        !IN_RANGE(in->min_factor, MIN_FACTOR_MIN, MIN_FACTOR_MAX) ||
        !IN_RANGE(in->max_factor, MAX_FACTOR_MIN, MAX_FACTOR_MAX) ||
        !IN_RANGE(in->speed_threshold, SPEED_THRESHOLD_MIN, SPEED_THRESHOLD_MAX) ||
        !IN_RANGE(in->speed_max, SPEED_MAX_MIN, SPEED_MAX_MAX) ||
        !IN_RANGE(in->y_boost, 1000, 3000) || in->y_boost % 10 ||
        in->min_factor > in->max_factor || in->speed_threshold >= in->speed_max ||
        in->curve > ACCEL_CURVE_SMOOTH || ACCEL_SMOOTH_FIELDS(INVALID)
        in->high_gain < in->low_gain || in->ramp_width > in->mid_speed) {
        return -ERANGE;
    }
#undef INVALID

    k_spinlock_key_t key = k_spin_lock(&config_lock);
    struct accel_config candidate = *cfg;
    k_spin_unlock(&config_lock, key);
    candidate.cfg.level2.sensitivity = in->sensitivity;
    candidate.cfg.level2.min_factor = in->min_factor;
    candidate.cfg.level2.max_factor = in->max_factor;
    candidate.cfg.level2.speed_threshold = in->speed_threshold;
    candidate.cfg.level2.speed_max = in->speed_max;
    candidate.cfg.level2.curve = in->curve;
#define COPY(field, lo, hi) candidate.cfg.level2.field = in->field;
    ACCEL_SMOOTH_FIELDS(COPY)
#undef COPY
    candidate.y_boost_scaled = (in->y_boost - 1000) / 10;
    int ret = accel_validate_config(&candidate);
    if (ret < 0) {
        return ret == ACCEL_ERR_OUT_OF_RANGE || ret == ACCEL_ERR_OVERFLOW ? -ERANGE : -EINVAL;
    }

    struct accel_data *data = dev->data;
    key = k_spin_lock(&config_lock);
    if (memcmp(&cfg->cfg.level2, &candidate.cfg.level2, sizeof(cfg->cfg.level2)) ||
        cfg->y_boost_scaled != candidate.y_boost_scaled) {
        cfg->cfg.level2 = candidate.cfg.level2;
        cfg->y_boost_scaled = candidate.y_boost_scaled;
        data->last_time_ms = k_uptime_get_32();
        data->recent_speed = 0;
#if defined(ACCEL_HAS_SMOOTH)
        data->smooth = (struct accel_smooth_data){0};
#endif
    }
    k_spin_unlock(&config_lock, key);
    return 0;
}
#endif


#if defined(ACCEL_HAS_SMOOTH)
#define SMOOTH_SPEED_SCALE 16
#define SMOOTH_GAIN_SHIFT 10

static uint32_t integer_sqrt(uint64_t n) {
    uint64_t root = 0;
    uint64_t bit = UINT64_C(1) << 62;
    while (bit > n) {
        bit >>= 2;
    }
    for (; bit; bit >>= 2) {
        if (n >= root + bit) {
            n -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
    }
    return root;
}

static uint16_t smooth_gain(const struct accel_config *cfg, int64_t speed) {
    const int32_t width = cfg->cfg.level2.ramp_width * SMOOTH_SPEED_SCALE;
    const int32_t start = cfg->cfg.level2.mid_speed * SMOOTH_SPEED_SCALE - width;
    const int64_t offset = speed - start;
    if (offset <= 0) {
        return cfg->cfg.level2.low_gain;
    }
    if (offset >= 2 * width) {
        return cfg->cfg.level2.high_gain;
    }
    const uint32_t one = BIT(SMOOTH_GAIN_SHIFT);
    const uint32_t t = ((uint32_t)offset << SMOOTH_GAIN_SHIFT) / (2 * width);
    const uint32_t s = (t * t * (3 * one - 2 * t)) >> (2 * SMOOTH_GAIN_SHIFT);
    return cfg->cfg.level2.low_gain +
           (((cfg->cfg.level2.high_gain - cfg->cfg.level2.low_gain) * s) >> SMOOTH_GAIN_SHIFT);
}

static void smooth_event(const struct accel_config *cfg, struct accel_smooth_data *data,
                         struct input_event *event) {
    const bool movement = event->type == INPUT_EV_REL && event->value != 0 &&
                          (event->code == INPUT_REL_X || event->code == INPUT_REL_Y);
    if (!movement && !event->sync) {
        return;
    }
    const uint32_t now = k_uptime_get_32();
    if (!data->frame_open && now - data->last_frame_ms > 100) {
        data->speed = 0;
        data->remainder[0] = 0;
        data->remainder[1] = 0;
    }
    if (movement) {
        if (!data->frame_open) {
            data->gain = smooth_gain(cfg, data->speed);
            data->frame_open = true;
        }
        const unsigned axis = event->code == INPUT_REL_Y;
        const uint32_t magnitude = event->value < 0 ? -(int64_t)event->value : event->value;
        // Saturate the accumulator, not the input; both squares then fit in uint64_t.
        data->magnitude[axis] += MIN(magnitude, (uint32_t)INT32_MAX - data->magnitude[axis]);
        uint32_t gain = data->gain;
        if (axis) {
            gain = gain * accel_decode_y_boost(cfg->y_boost_scaled) / SENSITIVITY_SCALE;
        }
        const int64_t scaled = (int64_t)event->value * gain + data->remainder[axis];
        const int64_t output = scaled / SENSITIVITY_SCALE;
        data->remainder[axis] = scaled - output * SENSITIVITY_SCALE;
        event->value = CLAMP(output, INT16_MIN, INT16_MAX);
    }
    if (event->sync) {
        const uint32_t x = data->magnitude[0], y = data->magnitude[1];
        const uint32_t magnitude = x && y ? integer_sqrt((uint64_t)x * x + (uint64_t)y * y) : x + y;
        data->speed += ((int64_t)magnitude * SMOOTH_SPEED_SCALE - data->speed) /
                       cfg->cfg.level2.smoothing;
        data->magnitude[0] = 0;
        data->magnitude[1] = 0;
        data->last_frame_ms = now;
        data->frame_open = false;
    }
}
#endif

// =============================================================================
// MAIN EVENT HANDLER (OPTIMIZED FOR EMBEDDED KEYBOARDS)
// =============================================================================

#if defined(ACCEL_HAS_SMOOTH)
static int legacy_event(const struct accel_config *cfg, struct accel_data *data,
                        struct input_event *event) {
#else
int accel_handle_event(const struct device *dev, struct input_event *event,
                      uint32_t param1, uint32_t param2,
                      struct zmk_input_processor_state *state) {
    if (!dev || !event || !dev->config || !dev->data) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;
#endif
    // Fast path: check event type and code
    if (event->type != cfg->input_type || event->value == 0) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    
    // Check supported axes (optimized for keyboard devices)
    if (event->code != INPUT_REL_X && event->code != INPUT_REL_Y && 
        event->code != INPUT_REL_WHEEL && event->code != INPUT_REL_HWHEEL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    
    // Input validation (minimal for embedded)
    int32_t input_value = event->value;
    if (abs(input_value) > MAX_INPUT_VALUE) {
        input_value = (input_value > 0) ? MAX_INPUT_VALUE : -MAX_INPUT_VALUE;
    }
    
    // Fast calculation dispatch
    int32_t result;
    if (cfg->level == 1) {
        result = accel_simple_calculate(cfg, input_value, event->code);
    } else {
        result = accel_standard_calculate(cfg, data, input_value, event->code);
    }
    
    // Emergency brake (simplified). Level 2 works in int32 on an input clamped to
    // +-MAX_INPUT_VALUE, so its output is bounded and only saturated to int16 below.
    if (cfg->level == 1 && abs(result) > EMERGENCY_LIMIT) {
        result = (result > 0) ? EMERGENCY_LIMIT : -EMERGENCY_LIMIT;
    }
    
    // Ensure minimum movement
    if (input_value != 0 && result == 0) {
        result = (input_value > 0) ? 1 : -1;
    }
    
    // Final clamp and update
    if (result > INT16_MAX) result = INT16_MAX;
    if (result < INT16_MIN) result = INT16_MIN;
    
    event->value = result;
    return ZMK_INPUT_PROC_CONTINUE;
}

#if defined(ACCEL_HAS_SMOOTH)
int accel_handle_event(const struct device *dev, struct input_event *event,
                      uint32_t param1, uint32_t param2,
                      struct zmk_input_processor_state *state) {
    if (!dev || !event || !dev->config || !dev->data) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;
    if (cfg->level == 2) {
        k_spinlock_key_t key = k_spin_lock(&config_lock);
        int ret = ZMK_INPUT_PROC_CONTINUE;
        if (cfg->cfg.level2.curve == ACCEL_CURVE_SMOOTH) {
            smooth_event(cfg, &data->smooth, event);
        } else {
            ret = legacy_event(cfg, data, event);
        }
        k_spin_unlock(&config_lock, key);
        return ret;
    }
    return legacy_event(cfg, data, event);
}
#endif

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)