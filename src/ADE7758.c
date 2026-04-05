#include "main.h"
#include "ADE7758.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"

spi_device_handle_t spi2;
SemaphoreHandle_t ready_sem; ///< Semaphore for ready signal

static void IRAM_ATTR irq_isr_handler(void *arg)
{
    /* xHigherPriorityTaskWoken must be set to pdFALSE before it is used. */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // xTaskNotifyFromISR(xTaskI2C, NOTYFY_SENSOR_SET_MAGACC_INT, eSetBits, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(ready_sem, &xHigherPriorityTaskWoken);

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
esp_err_t SPI_write(spi_device_handle_t handle, uint8_t addr, size_t length, uint8_t *data)
{
    esp_err_t err = ESP_FAIL;
    spi_transaction_t tx = {0};

    gpio_set_level(GPIO_NUM_15, 0);

    addr |= WRITE;
    tx.length = 8;
    tx.tx_buffer = &addr;
    ESP_ERROR_CHECK(spi_device_transmit(handle, &tx));

    uint8_t *txptr = data;

    for (int i = 0; i < length; i = i + 8)
    {
        memset(&tx, 0, sizeof(tx));
        tx.length = 8;
        tx.tx_buffer = txptr++;
        err = spi_device_transmit(handle, &tx);
        ESP_ERROR_CHECK(err);
    }

    gpio_set_level(GPIO_NUM_15, 1);

    return err;
};

esp_err_t SPI_read(spi_device_handle_t handle, uint8_t addr, size_t length, uint8_t *data)
{
    esp_err_t err = ESP_FAIL;
    spi_transaction_t tx = {0};
    spi_transaction_t rx = {0};

    gpio_set_level(GPIO_NUM_15, 0);
    tx.length = 8;
    tx.tx_buffer = &addr;
    ESP_ERROR_CHECK(spi_device_transmit(handle, &tx));

    rx.length = length;
    rx.rx_buffer = data;
    err = spi_device_transmit(handle, &rx);
    ESP_ERROR_CHECK(err);

    gpio_set_level(GPIO_NUM_15, 1);

    return err;
};

void testpin_task(void *arg)
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

void powermeter_task(void *arg)
{

    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_NUM_12,
        .mosi_io_num = GPIO_NUM_13,
        .sclk_io_num = GPIO_NUM_14,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 8};

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_10M, // Clock out at 10 MHz
        .mode = 1,                             // SPI mode 0
        .spics_io_num = GPIO_NUM_NC,           // CS pin
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .queue_size = 4,
        .flags = 0,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi2);
    ESP_ERROR_CHECK(ret);

    ready_sem = xSemaphoreCreateBinary();

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = BIT64(GPIO_NUM_27);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_install_isr_service(0);
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_NUM_27, irq_isr_handler, NULL));

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = BIT64(GPIO_NUM_15);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(GPIO_NUM_15, 1);

    uint8_t buf[64];

    int irqflags = ZXC;

    buf[0] = irqflags >> 16;
    buf[1] = irqflags >> 8;
    buf[2] = irqflags;

    // configure
    SPI_write(spi2, MASK, 24, buf);

    int32_t vrms[3];
    int32_t avgvrms[3];
    int32_t irms[3];
    int32_t avgirms[3];
    int avgcnt = 0;
    int16_t watth[3];

    int64_t time1 = esp_timer_get_time();
    int64_t time2 = esp_timer_get_time();

    int64_t cycle = 200000;

    while (1)
    {
        BaseType_t inter = xSemaphoreTake(ready_sem, 500 / portTICK_PERIOD_MS);
        time2 = esp_timer_get_time();

        SPI_read(spi2, AWATTHR, 16, buf);
        watth[0] = (buf[0] << 8) | (buf[1]);
        SPI_read(spi2, BWATTHR, 16, buf);
        watth[1] = (buf[0] << 8) | (buf[1]);
        SPI_read(spi2, CWATTHR, 16, buf);
        watth[2] = (buf[0] << 8) | (buf[1]);

        SPI_read(spi2, AIRMS, 24, buf);
        irms[0] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8);
        SPI_read(spi2, BIRMS, 24, buf);
        irms[1] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8);
        SPI_read(spi2, CIRMS, 24, buf);
        irms[2] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8);

        SPI_read(spi2, AVRMS, 24, buf);
        vrms[0] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8);
        SPI_read(spi2, BVRMS, 24, buf);
        vrms[1] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8);
        SPI_read(spi2, CVRMS, 24, buf);
        vrms[2] = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8);

        SPI_read(spi2, RSTATUS, 24, buf);
        irqflags = (buf[0] << 16) | (buf[1] << 8) | (buf[2]);

        avgcnt++;

        avgirms[0] += irms[0] / 256;
        avgirms[1] += irms[1] / 256;
        avgirms[2] += irms[2] / 256;

        avgvrms[0] += vrms[0] / 1254;
        avgvrms[1] += vrms[1] / 1254;
        avgvrms[2] += vrms[2] / 1254;

        if (time2 - time1 < cycle - 10000)
            continue;

        /*
                if (inter == pdTRUE)
                {
                    // ESP_LOGI("ADE7758", "Read RSTATUS: 0x%06x", irqflags);
                    ESP_LOGI("ADE7758", "Read WattHours: %i, %i, %i", watth[0], watth[1], watth[2]);
                    ESP_LOGI("ADE7758", "Read IRMS: %i, %i, %i", avgirms[0] / avgcnt / 256, avgirms[1] / avgcnt / 256, avgirms[2] / avgcnt / 256);
                    ESP_LOGI("ADE7758", "Read VRMS: %i, %i, %i", avgvrms[0] / avgcnt / 1254, avgvrms[1] / avgcnt / 1254, avgvrms[2] / avgcnt / 1254);
                }
                else
                {
                    ESP_LOGW("ADE7758", "Not IRQ. Read VRMS: %i, %i, %i", vrms[0] / 1254, vrms[1] / 1254, vrms[2] / 1254);
                }
        */

        ESP_LOGI("ADE7758", "Cycle delta: %lli", time2 - time1 - cycle);
        ESP_LOGI("ADE7758", "Read WattHours: %i, %i, %i", watth[0], watth[1], watth[2]);
        ESP_LOGI("ADE7758", "Read IRMS: %i, %i, %i", avgirms[0] / avgcnt, avgirms[1] / avgcnt, avgirms[2] / avgcnt);
        ESP_LOGI("ADE7758", "Read VRMS: %i, %i, %i", avgvrms[0] / avgcnt, avgvrms[1] / avgcnt, avgvrms[2] / avgcnt);

        avgvrms[0] = 0;
        avgvrms[1] = 0;
        avgvrms[2] = 0;

        avgirms[0] = 0;
        avgirms[1] = 0;
        avgirms[2] = 0;

        avgcnt = 0;

        time1 = time2;
    }
}