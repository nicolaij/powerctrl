#include "main.h"
#include "driver/spi_master.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "math.h"
#include "pid_ctrl.h"

spi_device_handle_t spi3;

SemaphoreHandle_t ready_sem2; ///< Semaphore for ready signal

static void IRAM_ATTR irq_isr_handler(void *arg)
{
    /* xHigherPriorityTaskWoken must be set to pdFALSE before it is used. */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // xTaskNotifyFromISR(xTaskI2C, NOTYFY_SENSOR_SET_MAGACC_INT, eSetBits, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(ready_sem2, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        /* Writing to the queue caused a task to unblock and the unblocked task
           has a priority higher than or equal to the priority of the currently
           executing task (the task this interrupt interrupted). Perform a
           context switch so this interrupt returns directly to the unblocked
           task. */
        portYIELD_FROM_ISR(); /* or portEND_SWITCHING_ISR() depending on the
                                 port.*/
    }
}

esp_err_t SPI3_write(spi_device_handle_t handle, uint8_t addr, size_t length, uint8_t data)
{
    esp_err_t err = ESP_FAIL;
    spi_transaction_t tx = {0};

    gpio_set_level(GPIO_NUM_5, 0);

    tx.addr = addr | 0x80;
    tx.length = 8;
    tx.tx_buffer = &data;
    err = spi_device_transmit(handle, &tx);
    ESP_ERROR_CHECK(err);

    gpio_set_level(GPIO_NUM_5, 1);

    return err;
};

esp_err_t SPI3_read(spi_device_handle_t handle, uint8_t addr, size_t length, uint8_t *data)
{
    esp_err_t err = ESP_FAIL;
    spi_transaction_t rx = {0};

    gpio_set_level(GPIO_NUM_5, 0);

    rx.addr = addr;
    rx.length = length;
    rx.rx_buffer = data;
    err = spi_device_transmit(handle, &rx);
    ESP_ERROR_CHECK(err);

    gpio_set_level(GPIO_NUM_5, 1);

    return err;
};

void rtdtestpin_task(void *arg)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = BIT64(GPIO_NUM_15) | BIT64(GPIO_NUM_14) | BIT64(GPIO_NUM_13);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    while (1)
    {
        gpio_set_level(GPIO_NUM_13, 0);
        gpio_set_level(GPIO_NUM_14, 0);
        gpio_set_level(GPIO_NUM_15, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_13, 1);
        gpio_set_level(GPIO_NUM_14, 1);
        gpio_set_level(GPIO_NUM_15, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    };
}

void rtd_task(void *arg)
{

    int Cycle = (int)get_menu_val_by_id("TCycle");
    int Enable = (int)get_menu_val_by_id("TpidEna");

    pid_ctrl_block_t pidT;
    pid_ctrl_block_handle_t pid_handleT = &pidT;

    pid_ctrl_config_t pid_conf = {.init_param.cal_type = PID_CAL_TYPE_POSITIONAL,
                                  .init_param.kp = get_menu_val_by_id("TpidP"),
                                  .init_param.ki = get_menu_val_by_id("TpidI"),
                                  .init_param.kd = get_menu_val_by_id("TpidD"),
                                  .init_param.max_integral = get_menu_val_by_id("TpidMax"),
                                  .init_param.min_integral = 0,
                                  .init_param.max_output = get_menu_val_by_id("TpidMax"),
                                  .init_param.min_output = 0};

    ESP_ERROR_CHECK(pid_update_parameters(pid_handleT, &pid_conf.init_param));

    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_NUM_19,
        .mosi_io_num = GPIO_NUM_23,
        .sclk_io_num = GPIO_NUM_18,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 8};

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_10M / 2, // Clock out at 5 MHz
        .mode = 1,                                 // SPI mode 1
        .spics_io_num = GPIO_NUM_NC,               // CS pin
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .queue_size = 4,
        .flags = 0,
        .command_bits = 0,
        .address_bits = 8,
        .dummy_bits = 0,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi3);
    ESP_ERROR_CHECK(ret);

    ready_sem2 = xSemaphoreCreateBinary();

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = BIT64(GPIO_NUM_21);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_install_isr_service(0);
    // ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_NUM_21, irq_isr_handler, NULL));

    // CS
    gpio_set_level(GPIO_NUM_5, 1);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = BIT64(GPIO_NUM_5);
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // vTaskDelay(1000);

    uint8_t buf[4];

    int64_t time1 = esp_timer_get_time();
    int64_t time2 = esp_timer_get_time();

    int ready = 0;
    int res = 0;

    while (1)
    {
        if (ready == 0)
        {
            // Configure
            /*
            BIAS (D7)
            Conversion Mode (D6) 1 = Auto
            Fault Status Clear (D1)
            50/60Hz (D0) 1 to this bit to reject 50Hz
            Fault detection with automatic delay (D2)
            */
            uint8_t conf = (1 << 7) | (1 << 2) | (1); // Fault detection with automatic delay
            SPI3_write(spi3, 0, 8, conf);

            vTaskDelay(100 / portTICK_PERIOD_MS);

            if (SPI3_read(spi3, 7, 8, buf) == ESP_OK)
            {
                if (buf[0] > 0)
                {
                    ESP_LOGW("RTD", "Fault = %02x", buf[0]);
                    conf = (1 << 1) | (1); // Fault Status Clear
                    SPI3_write(spi3, 0, 8, conf);
                }
                else
                {
                    ready = 1;
                }
            };
        }

        BaseType_t inter = xSemaphoreTake(ready_sem2, Cycle / portTICK_PERIOD_MS);
        time2 = esp_timer_get_time();

        uint8_t conf = (1 << 7) | (1 << 5) | (1); // 1-shot
        SPI3_write(spi3, 0, 8, conf);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        if (SPI3_read(spi3, 1, 16, buf) == ESP_OK)
        {
            res = (buf[0] << 8) | buf[1];

            //ESP_LOGD("RTD", "Read %d", res);
        };

        if (res == 0 || (res & 1) != 0)
        {
            ESP_LOGW("RTD", "Fault");
            ready = 0;
        }
        else
        {
            // float t = (((res >> 1) * 430) / 32768.0f - 100.0f) / (100.0f * 0.00385f);
            float t2 = ((res / 32) - 256) / 10.0f;
            ESP_LOGI("RTD", "T = %f", t2);

            input[10] = t2 * 10.0f;

            float input_error = input[10] - holding[10];
            float ret_result = 0;

            ESP_ERROR_CHECK(pid_compute(pid_handleT, input_error, &ret_result));

            // если регулятор включен
            if (holding[9] & BIT3)
            {
                holding[3] = ret_result;
                holding[4] = ret_result;
                holding[5] = ret_result;
            }
            else
                pid_reset_ctrl_block(pid_handleT);
        }

        time1 = time2;
    }
}