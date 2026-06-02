#include <main.h>
#include "mbcontroller.h" // for mbcontroller defines and api
#include "driver/gptimer.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

TaskHandle_t xHandleWifi = NULL;
TaskHandle_t xHandlePower = NULL;
TaskHandle_t xHandleRTD = NULL;
TaskHandle_t xHandleConsole = NULL;
TaskHandle_t xHandleDallas = NULL;

static void *mbc_slave_handle = NULL;

/*
0 - задание мощности для фазы A (0 - 100)%
1 - задание мощности для фазы B (0 - 100)%
2 - задание мощности для фазы C (0 - 100)%
3 - задание мощности регулятора P для фазы А (0 - 32000)W
4 - задание мощности регулятора P для фазы B (0 - 32000)W
5 - задание мощности регулятора P для фазы C (0 - 32000)W

9 - биты включение регуляторов 0:PA, 1:PB, 2:PC; 3:T
10 - задание температуры нагревателя (RTD) (0 - 1000) 0.1С
*/
int16_t holding[16];

/*
0 - измеренный ток для фазы A (0 - 10000) 100А (0.01А)
1 - измеренный ток для фазы B (0 - 10000) 100А (0.01А)
2 - измеренный ток для фазы C (0 - 10000) 100А (0.01А)
3 - измеренное напряжение для фазы A (0 - 2500) 250В (0.1В)
4 - измеренное напряжение для фазы B (0 - 2500) 250В (0.1В)
5 - измеренное напряжение для фазы C (0 - 2500) 250В (0.1В)
6 - измеренная мощность для фазы A (0 - 32000)W 32kW (1W)
7 - измеренная мощность для фазы A (0 - 32000)W 32kW (1W)
8 - измеренная мощность для фазы A (0 - 32000)W 32kW (1W)
9 -
10 - температура нагревателя (RTD) (0 - 1000) 0.1С
*/
int16_t input[16];

// Расчет по Алгоритму Брезенхема. Прерывание 10мс
static bool IRAM_ATTR gptimer_callback10(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    const int cycle_count = 100; //*(int *)user_data;

    static int counter_ = 0;
    static int v1_[3] = {0, 0, 0};
    static int v0_[3] = {0, 0, 0};

    counter_++;

    int countA = cycle_count * holding[0] / 100;
    v1_[0] = counter_ * countA / cycle_count;

    if (v1_[0] == v0_[0])
        gpio_set_level(OUTA_PIN, 1);
    else
        gpio_set_level(OUTA_PIN, 0);

    int countB = cycle_count * holding[1] / 100;
    v1_[1] = counter_ * countB / cycle_count;

    if (v1_[1] == v0_[1])
        gpio_set_level(OUTB_PIN, 1);
    else
        gpio_set_level(OUTB_PIN, 0);

    int countC = cycle_count * holding[2] / 100;
    v1_[2] = counter_ * countC / cycle_count;

    if (v1_[2] == v0_[2])
        gpio_set_level(OUTC_PIN, 1);
    else
        gpio_set_level(OUTC_PIN, 0);

    v0_[0] = v1_[0];
    v0_[1] = v1_[1];
    v0_[2] = v1_[2];

    if (counter_ >= cycle_count)
    {
        counter_ = 0;
        v0_[0] = 0;
        v0_[1] = 0;
        v0_[2] = 0;
    }

    return true; // Auto-reload the timer automatically
};

void app_main()
{

    init_nvs();
    read_nvs_menu();

    adc_channel_t adc_channel;
    adc_unit_t adc_unit;

    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(PRESSURE_PIN, &adc_unit, &adc_channel));

    ESP_LOGI("ADC", "Unit: %i, Channel: %i", adc_unit, adc_channel);

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = adc_unit,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t oneshot_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_channel, &oneshot_config));

    adc_cali_handle_t cali_handle = NULL;

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = adc_unit,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));

    xTaskCreate(powermeter_task, "powermeter_task", 1024 * 4, NULL, configMAX_PRIORITIES - 10, &xHandlePower);
    xTaskCreate(rtd_task, "rtd_task", 1024 * 4, NULL, configMAX_PRIORITIES - 12, &xHandleRTD);
    xTaskCreate(btn_task, "btn_task", 1024 * 4, NULL, configMAX_PRIORITIES - 15, NULL);
    xTaskCreate(console_task, "console_task", 1024 * 4, NULL, configMAX_PRIORITIES - 16, &xHandleConsole);
    xTaskCreate(wifi_task, "wifi_task", 1024 * 3, NULL, configMAX_PRIORITIES - 12, &xHandleWifi);
    xTaskCreate(dallas_task, "dallas_task", 1024 * 3, NULL, configMAX_PRIORITIES - 13, &xHandleDallas);

    memset(holding, 0, sizeof(holding));
    memset(input, 0, sizeof(input));

    int Cycle = (int)get_menu_val_by_id("Cycle");

    mb_param_info_t reg_info;                     // keeps the Modbus registers access information
    mb_register_area_descriptor_t reg_area = {0}; // Modbus register area descriptor structure

    // Initialize Modbus controller
    mb_communication_info_t comm_config = {
        .ser_opts.port = UART_NUM_2,
        .ser_opts.mode = MB_RTU,
        .ser_opts.baudrate = 115200,
        .ser_opts.parity = MB_PARITY_NONE,
        .ser_opts.uid = 1,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_1};

    ESP_ERROR_CHECK(mbc_slave_create_serial(&comm_config, &mbc_slave_handle)); // Initialization of Modbus controller

    // Initialization of Input Registers area
    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = 0;
    reg_area.address = input;
    reg_area.size = sizeof(input);
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(mbc_slave_handle, reg_area));

    reg_area.type = MB_PARAM_HOLDING; // Set type of register area
    reg_area.start_offset = 0;        // Offset of register area in Modbus protocol
    reg_area.address = holding;       // Set pointer to storage instance
    // Set the size of register storage instance in bytes
    reg_area.size = sizeof(holding);
    reg_area.access = MB_ACCESS_RW;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(mbc_slave_handle, reg_area));

    // Set UART pin numbers
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, GPIO_NUM_17,
                                 GPIO_NUM_16, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    // esp_err_t err = mbc_slave_start(mbc_slave_handle);

    // setup OUT
    gpio_set_level(OUTA_PIN, 1);
    gpio_set_level(OUTB_PIN, 1);
    gpio_set_level(OUTC_PIN, 1);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = BIT64(OUTA_PIN) | BIT64(OUTB_PIN) | BIT64(OUTC_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = gptimer_callback10,
    };

    int cycle_count = Cycle / 10;

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, (void *)&cycle_count));

    gptimer_alarm_config_t alarm_config1 = {
        .reload_count = 0,
        .alarm_count = 10000, // period = 10ms
        .flags.auto_reload_on_alarm = true};

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));

    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    int adc_raw, voltage;

    while (1)
    {
        int adc_sum = 0;
        for (int i = 0; i < 20; i++)
        {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc_channel, &adc_raw));
            vTaskDelay(1);
            adc_sum += adc_raw;
        }
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, adc_sum / 20, &voltage));
        const int pressureSensorOffset = 500;
        const float pressureSensorSens = 4.0f / 3000.0f;
        if (voltage * 2 < pressureSensorOffset)
        {
            ESP_LOGW("ADC", "Pressure < 0 (%i mV). Stop!", voltage * 2);
            // holding[9] = 0;
            // holding[0] = 0;
            // holding[1] = 0;
            // holding[2] = 0;
        }
        else
        {
            float pressure = (voltage * 2 - pressureSensorOffset) * pressureSensorSens;
            ESP_LOGI("ADC", "Pressure = %i mV, %.1f bar", voltage * 2, pressure);
            if (pressure > 2.8f || pressure < 0.8f)
            {
                // holding[9] = 0;
                // holding[0] = 0;
                // holding[1] = 0;
                // holding[2] = 0;
                ESP_LOGW("ADC", "Pressure! Stop!", voltage * 2);
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    };
}