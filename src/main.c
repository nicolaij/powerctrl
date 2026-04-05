#include <main.h>
#include "mbcontroller.h" // for mbcontroller defines and api

TaskHandle_t xHandleWifi = NULL;
TaskHandle_t xHandleADC = NULL;
TaskHandle_t xHandlePower = NULL;
TaskHandle_t xHandleConsole = NULL;

static void *mbc_slave_handle = NULL;
int16_t holding[128];

void app_main()
{

    init_nvs();
    read_nvs_menu();

    xTaskCreate(powermeter_task, "powermeter_task", 1024 * 4, NULL, configMAX_PRIORITIES - 10, &xHandlePower);
    //xTaskCreate(testpin_task, "powermeter_task", 1024 * 4, NULL, configMAX_PRIORITIES - 10, &xHandlePower);
    xTaskCreate(btn_task, "btn_task", 1024 * 4, NULL, configMAX_PRIORITIES - 15, NULL);
    xTaskCreate(console_task, "console_task", 1024 * 4, NULL, configMAX_PRIORITIES - 16, &xHandleConsole);
    xTaskCreate(wifi_task, "wifi_task", 1024 * 3, NULL, configMAX_PRIORITIES - 12, &xHandleWifi);

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

    while (1)
    {
        vTaskDelay(100);
    };
}