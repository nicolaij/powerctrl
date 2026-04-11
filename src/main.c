#include <main.h>
#include "mbcontroller.h" // for mbcontroller defines and api
#include "driver/gptimer.h"

TaskHandle_t xHandleWifi = NULL;
TaskHandle_t xHandleADC = NULL;
TaskHandle_t xHandlePower = NULL;
TaskHandle_t xHandleConsole = NULL;

static void *mbc_slave_handle = NULL;
int16_t holding[128];

// Расчет по Алгоритму Брезенхема. Прерывание 10мс
static bool IRAM_ATTR gptimer_callback10(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    const int cycle_count = 100;//*(int *)user_data;

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

    xTaskCreate(powermeter_task, "powermeter_task", 1024 * 4, NULL, configMAX_PRIORITIES - 10, &xHandlePower);
    // xTaskCreate(testpin_task, "powermeter_task", 1024 * 4, NULL, configMAX_PRIORITIES - 10, &xHandlePower);
    xTaskCreate(btn_task, "btn_task", 1024 * 4, NULL, configMAX_PRIORITIES - 15, NULL);
    xTaskCreate(console_task, "console_task", 1024 * 4, NULL, configMAX_PRIORITIES - 16, &xHandleConsole);
    xTaskCreate(wifi_task, "wifi_task", 1024 * 3, NULL, configMAX_PRIORITIES - 12, &xHandleWifi);

    memset(holding, 0, sizeof(holding));

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

    //esp_err_t err = mbc_slave_start(mbc_slave_handle);

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

    while (1)
    {
        vTaskDelay(100);
    };
}