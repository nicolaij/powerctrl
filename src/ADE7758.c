#include "main.h"
#include "ADE7758.h"
#include "driver/spi_master.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "math.h"
#include "pid_ctrl.h"

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

uint32_t uint24(uint8_t r1, uint8_t r2, uint8_t r3)
{
    return (r1 << 16) | (r2 << 8) | (r3);
}

int32_t int24(uint8_t r1, uint8_t r2, uint8_t r3)
{
    if (r1 & 0x80)
    {
        return 0xFF000000 | (r1 << 16) | (r2 << 8) | (r3);
    }
    else
    {
        return (r1 << 16) | (r2 << 8) | (r3);
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

    // Reset
    buf[0] = SWRST;
    SPI_write(spi2, OPMODE, 8, buf);

    int IGain = (int)get_menu_val_by_id("IGain");
    int UGain = (int)get_menu_val_by_id("UGain");
    int Cycle = (int)get_menu_val_by_id("Cycle");
    int AIGain = (int)get_menu_val_by_id("AIGain");
    int BIGain = (int)get_menu_val_by_id("BIGain");
    int CIGain = (int)get_menu_val_by_id("CIGain");
    int AVRMSGain = (int)get_menu_val_by_id("AVRMSGain");
    int BVRMSGain = (int)get_menu_val_by_id("BVRMSGain");
    int CVRMSGain = (int)get_menu_val_by_id("CVRMSGain");

    int16_t AIOffset = (int16_t)get_menu_val_by_id("AIOffset");
    int16_t BIOffset = (int16_t)get_menu_val_by_id("BIOffset");
    int16_t CIOffset = (int16_t)get_menu_val_by_id("CIOffset");

    int16_t AVOffset = (int16_t)get_menu_val_by_id("AVOffset");
    int16_t BVOffset = (int16_t)get_menu_val_by_id("BVOffset");
    int16_t CVOffset = (int16_t)get_menu_val_by_id("CVOffset");

    float IRange = get_menu_val_by_id("IRange");
    float URange = get_menu_val_by_id("URange");

    pid_ctrl_block_t pidA;
    pid_ctrl_block_handle_t pid_handleA = &pidA;
    pid_ctrl_block_t pidB;
    pid_ctrl_block_handle_t pid_handleB = &pidB;
    pid_ctrl_block_t pidC;
    pid_ctrl_block_handle_t pid_handleC = &pidC;

    pid_ctrl_config_t pid_conf = {.init_param.cal_type = PID_CAL_TYPE_POSITIONAL,
                                  .init_param.kp = get_menu_val_by_id("pidP"),
                                  .init_param.ki = get_menu_val_by_id("pidI"),
                                  .init_param.kd = get_menu_val_by_id("pidD"),
                                  .init_param.max_integral = 100,
                                  .init_param.min_integral = 0,
                                  .init_param.max_output = 100,
                                  .init_param.min_output = 0};

    ESP_ERROR_CHECK(pid_update_parameters(pid_handleA, &pid_conf.init_param));
    ESP_ERROR_CHECK(pid_update_parameters(pid_handleB, &pid_conf.init_param));
    ESP_ERROR_CHECK(pid_update_parameters(pid_handleC, &pid_conf.init_param));

    int irqflags = ZXA | ZXB | ZXC;

    buf[0] = irqflags >> 16;
    buf[1] = irqflags >> 8;
    buf[2] = irqflags;

    // configure IRQ
    SPI_write(spi2, MASK, 24, buf);

    int gain = 0;

    if (IGain == 2)
    {
        gain |= 0b01;
    }
    else if (IGain == 4)
    {
        gain |= 0b10;
    }

    if (UGain == 2)
    {
        gain |= 0b0010 << 4;
    }
    else if (UGain == 4)
    {
        gain |= 0b0100 << 4;
    }

    if (gain)
    {
        buf[0] = gain;
        SPI_write(spi2, GAIN, 8, buf);
    }

    if (AIGain > 0)
    {
        buf[0] = AIGain >> 8;
        buf[1] = AIGain & 0xFF;
        SPI_write(spi2, AIGAIN, 16, buf);
    }
    if (BIGain > 0)
    {
        buf[0] = BIGain >> 8;
        buf[1] = BIGain & 0xFF;
        SPI_write(spi2, BIGAIN, 16, buf);
    }
    if (CIGain > 0)
    {
        buf[0] = CIGain >> 8;
        buf[1] = CIGain & 0xFF;
        SPI_write(spi2, CIGAIN, 16, buf);
    }
    if (AVRMSGain > 0)
    {
        buf[0] = AVRMSGain >> 8;
        buf[1] = AVRMSGain & 0xFF;
        SPI_write(spi2, AVRMSGAIN, 16, buf);
    }
    if (BVRMSGain > 0)
    {
        buf[0] = BVRMSGain >> 8;
        buf[1] = BVRMSGain & 0xFF;
        SPI_write(spi2, BVRMSGAIN, 16, buf);
    }
    if (CVRMSGain > 0)
    {
        buf[0] = CVRMSGain >> 8;
        buf[1] = CVRMSGain & 0xFF;
        SPI_write(spi2, CVRMSGAIN, 16, buf);
    }

    if (AIOffset != 0)
    {
        buf[0] = AIOffset >> 8;
        buf[1] = AIOffset & 0xFF;
        SPI_write(spi2, AIRMSOS, 16, buf);
    }
    if (BIOffset != 0)
    {
        buf[0] = BIOffset >> 8;
        buf[1] = BIOffset & 0xFF;
        SPI_write(spi2, BIRMSOS, 16, buf);
    }
    if (CIOffset != 0)
    {
        buf[0] = CIOffset >> 8;
        buf[1] = CIOffset & 0xFF;
        SPI_write(spi2, CIRMSOS, 16, buf);
    }

    if (AVOffset != 0)
    {
        buf[0] = AVOffset >> 8;
        buf[1] = AVOffset & 0xFF;
        SPI_write(spi2, AVRMSOS, 16, buf);
    }
    if (BVOffset != 0)
    {
        buf[0] = BVOffset >> 8;
        buf[1] = BVOffset & 0xFF;
        SPI_write(spi2, BVRMSOS, 16, buf);
    }
    if (CVOffset != 0)
    {
        buf[0] = CVOffset >> 8;
        buf[1] = CVOffset & 0xFF;
        SPI_write(spi2, CVRMSOS, 16, buf);
    }

    int32_t vrms[3];
    int32_t avgvrms[3];
    int32_t irms[3];
    int32_t avgirms[3];
    int avgcntA = 0;
    int avgcntB = 0;
    int avgcntC = 0;
    int16_t watth[3];
    int16_t avgwatth[3];

    int64_t time1 = esp_timer_get_time();
    int64_t time2 = esp_timer_get_time();

    int64_t cycle = Cycle * 1000;

    float fltirms[3];
    float fltvrms[3];
    float fltwrms[3];

    float ki = IRange / 1914753.0f;                 // 2642412.0f; //  100000 / 2642412
    float ku = URange / (1651972.0f * sqrtf(2.0f)); // 10056.0f; //  500000 / 10056
    float kw = 1.0;

    int avgcnt = Cycle / 20; // кол-во положительных переходов через 0

    while (1)
    {
        BaseType_t inter = xSemaphoreTake(ready_sem, Cycle / portTICK_PERIOD_MS * 5);
        time2 = esp_timer_get_time();

        int interruptstatus = 0;

        /*         SPI_read(spi2, MASK, 24, buf);
                ESP_LOGI("ADE7758", "MASK: %x", int24(buf[0], buf[1], buf[2]));
                SPI_read(spi2, STATUS, 24, buf);
                ESP_LOGI("ADE7758", "STATUS: %x", int24(buf[0], buf[1], buf[2])); */

        if (SPI_read(spi2, RSTATUS, 24, buf) == ESP_OK)
        {
            interruptstatus = uint24(buf[0], buf[1], buf[2]);
        };

        if (inter == pdFALSE)
        {
            interruptstatus = (ZXA | ZXB | ZXC);
        }

        // фаза А
        if (ZXA & interruptstatus)
        {
            SPI_read(spi2, AWATTHR, 16, buf);
            watth[0] = (buf[0] << 8) | (buf[1]);
            avgwatth[0] += watth[0];

            SPI_read(spi2, AIRMS, 24, buf);
            irms[0] = int24(buf[0], buf[1], buf[2]);
            avgirms[0] += irms[0];

            SPI_read(spi2, AVRMS, 24, buf);
            vrms[0] = int24(buf[0], buf[1], buf[2]);
            avgvrms[0] += vrms[0];

            avgcntA++;

            if (avgcntA >= avgcnt || inter == pdFALSE)
            {
                fltirms[0] = (avgirms[0] / avgcntA) * ki;
                fltvrms[0] = (avgvrms[0] / avgcntA) * ku;
                fltwrms[0] = (watth[0] / avgcntA) * kw;

                ESP_LOGD("ADE7758", "(%3i) Phase A: %.3f V, %.3f A, %.0f W", avgcntA, fltvrms[0], fltirms[0], fltwrms[0]);

                avgwatth[0] = 0;
                avgvrms[0] = 0;
                avgirms[0] = 0;
                avgcntA = 0;

                // текущая мощность
                holding[3] = (fltwrms[0] < 32000) ? (int16_t)fltwrms[0] : 32000;

                float input_error = holding[6] - holding[3];
                float ret_result = 0;

                ESP_ERROR_CHECK(pid_compute(pid_handleA, input_error, &ret_result));
                // регулятор включен
                if (holding[9] == 1)
                    holding[0] = ret_result;
                else
                    pid_reset_ctrl_block(pid_handleA);
            }
        }

        // фаза B
        if (ZXB & interruptstatus)
        {
            SPI_read(spi2, BWATTHR, 16, buf);
            watth[1] = (buf[0] << 8) | (buf[1]);
            avgwatth[1] += watth[1];

            SPI_read(spi2, BIRMS, 24, buf);
            irms[1] = int24(buf[0], buf[1], buf[2]);
            avgirms[1] += irms[1];

            SPI_read(spi2, BVRMS, 24, buf);
            vrms[1] = int24(buf[0], buf[1], buf[2]);
            avgvrms[1] += vrms[1];

            avgcntB++;

            if (avgcntB >= avgcnt || inter == pdFALSE)
            {
                fltirms[1] = (avgirms[1] / avgcntB) * ki;
                fltvrms[1] = (avgvrms[1] / avgcntB) * ku;
                fltwrms[1] = (watth[1] / avgcntB) * kw;

                ESP_LOGD("ADE7758", "(%3i) Phase B: %.3f V, %.3f A, %.0f W", avgcntB, fltvrms[1], fltirms[1], fltwrms[1]);

                avgwatth[1] = 0;
                avgvrms[1] = 0;
                avgirms[1] = 0;
                avgcntB = 0;

                // текущая мощность
                holding[4] = (fltwrms[1] < 32000) ? (int16_t)fltwrms[1] : 32000;

                float input_error = holding[7] - holding[4];
                float ret_result = 0;

                ESP_ERROR_CHECK(pid_compute(pid_handleB, input_error, &ret_result));
                // регулятор включен
                if (holding[10] == 1)
                    holding[1] = ret_result;
                else
                    pid_reset_ctrl_block(pid_handleB);
            }
        };

        // фаза C
        if (ZXC & interruptstatus)
        {
            SPI_read(spi2, CWATTHR, 16, buf);
            watth[2] = (buf[0] << 8) | (buf[1]);
            avgwatth[2] += watth[2];

            SPI_read(spi2, CIRMS, 24, buf);
            irms[2] = int24(buf[0], buf[1], buf[2]);
            avgirms[2] += irms[2];
            // ESP_LOGW("ADE7758", "Read IRMS: %02x%02x%02x", buf[0], buf[1], buf[2]);

            SPI_read(spi2, CVRMS, 24, buf);
            vrms[2] = int24(buf[0], buf[1], buf[2]);
            avgvrms[2] += vrms[2];

            avgcntC++;

            if (avgcntC >= avgcnt || inter == pdFALSE)
            {
                fltirms[2] = (avgirms[2] / avgcntC) * ki;
                fltvrms[2] = (avgvrms[2] / avgcntC) * ku;
                fltwrms[2] = (watth[2] / avgcntC) * kw;

                ESP_LOGD("ADE7758", "(%3i) Phase C: %.3f V, %.3f A, %.0f W", avgcntC, fltvrms[2], fltirms[2], fltwrms[2]);

                avgwatth[2] = 0;
                avgvrms[2] = 0;
                avgirms[2] = 0;
                avgcntC = 0;

                // текущая мощность
                holding[5] = (fltwrms[2] < 32000) ? (int16_t)fltwrms[2] : 32000;

                float input_error = holding[8] - holding[5];
                float ret_result = 0;

                ESP_ERROR_CHECK(pid_compute(pid_handleC, input_error, &ret_result));
                // регулятор включен
                if (holding[11] == 1)
                    holding[2] = ret_result;
                else
                    pid_reset_ctrl_block(pid_handleC);
            }
        }

        // ESP_LOGW("ADE7758", "Read VRMS: %x, %x, %x", vrms[0], vrms[1], vrms[2]);

        if (time2 - time1 < cycle - 3000)
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

        // CURRENT CHANNEL ADC range +-2,642,412

        ESP_LOGI("OUT", "%i, %i, %i; time: %lli", holding[0], holding[1], holding[2], time2 - time1);
        time1 = time2;
    }
}