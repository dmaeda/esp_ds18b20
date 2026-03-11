#include "esp_ds18b20.h"
#include "owb.h"
#include "owb_gpio.h"
#include "ds18b20.h"

//#define GPIO_DS18B20_0       (5)
#define MAX_DEVICES          (8)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD        (5000)   // milliseconds
#define DISCOVERY_RETRY_PERIOD (10000) // milliseconds
#define DS18B20_TEMP_MIN_C   (-55.0f)
#define DS18B20_TEMP_MAX_C   (125.0f)

static const char *TAG = "esp_ds18b20";
static float readings[MAX_DEVICES] = {0};


static SemaphoreHandle_t s_lock = NULL;
static int num_devices = 0;

static void lock_take(void)
{
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
}

static void lock_give(void)
{
    if (s_lock) xSemaphoreGive(s_lock);
}

void esp_ds18b20_task(void *pvParameters)
{
    (void)pvParameters;

    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            ESP_LOGE(TAG, "Failed to create DS18B20 mutex");
            vTaskDelete(NULL);
            return;
        }
    }

    // Override global log level
    esp_log_level_set("*", ESP_LOG_INFO);

    // To debug, use 'make menuconfig' to set default Log level to DEBUG, then uncomment:
    //esp_log_level_set("owb", ESP_LOG_DEBUG);
    //esp_log_level_set("ds18b20", ESP_LOG_DEBUG);

    // Stable readings require a brief period before communication
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1)
    {
        num_devices = 0;

        // Create a 1-Wire bus, using GPIO bit-banging
        OneWireBus *owb;
        owb_gpio_driver_info gpio_driver_info;
        owb = owb_gpio_initialize(&gpio_driver_info, GPIO_DS18B20_0);
        owb_use_crc(owb, true);  // enable CRC check for ROM code

        // Find all connected devices
        ESP_LOGI(TAG, "Scanning 1-Wire bus for DS18B20 devices");
        OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};

        OneWireBus_SearchState search_state = {0};
        bool found = false;
        owb_search_first(owb, &search_state, &found);
        while (found)
        {
            char rom_code_s[17];
            owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
            ESP_LOGI(TAG, "Device %d ROM: %s", num_devices, rom_code_s);
            device_rom_codes[num_devices] = search_state.rom_code;
            ++num_devices;
            owb_search_next(owb, &search_state, &found);
        }
        ESP_LOGI(TAG, "Found %d device%s", num_devices, num_devices == 1 ? "" : "s");

        if (num_devices <= 0)
        {
            lock_take();
            readings[0] = 0.0f;
            lock_give();
            owb_uninitialize(owb);
            ESP_LOGW(TAG, "No DS18B20 devices detected; retrying in %d ms", DISCOVERY_RETRY_PERIOD);
            vTaskDelay(pdMS_TO_TICKS(DISCOVERY_RETRY_PERIOD));
            continue;
        }

        // In this example, if a single device is present, then the ROM code is probably
        // not very interesting, so just print it out. If there are multiple devices,
        // then it may be useful to check that a specific device is present.
        if (num_devices == 1)
        {
            // For a single device only:
            OneWireBus_ROMCode rom_code;
            owb_status status = owb_read_rom(owb, &rom_code);
            if (status == OWB_STATUS_OK)
            {
                char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
                owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
                ESP_LOGI(TAG, "Single device %s present", rom_code_s);
            }
            else
            {
                ESP_LOGE(TAG, "Error reading ROM code: %d", status);
            }
        }
        else
        {
            // Search for a known ROM code (LSB first):
            // For example: 0x1502162ca5b2ee28
            OneWireBus_ROMCode known_device = {
                .fields.family = { 0x28 },
                .fields.serial_number = { 0xee, 0xb2, 0xa5, 0x2c, 0x16, 0x02 },
                .fields.crc = { 0x15 },
            };
            char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
            owb_string_from_rom_code(known_device, rom_code_s, sizeof(rom_code_s));
            bool is_present = false;

            owb_status search_status = owb_verify_rom(owb, known_device, &is_present);
            if (search_status == OWB_STATUS_OK)
            {
                ESP_LOGI(TAG, "Device %s is %s", rom_code_s, is_present ? "present" : "not present");
            }
            else
            {
                ESP_LOGW(TAG, "Error searching for known device: %d", search_status);
            }
        }

        // Create DS18B20 devices on the 1-Wire bus
        DS18B20_Info * devices[MAX_DEVICES] = {0};
        bool alloc_failed = false;
        for (int i = 0; i < num_devices; ++i)
        {
            DS18B20_Info * ds18b20_info = ds18b20_malloc();  // heap allocation
            if (ds18b20_info == NULL) {
                ESP_LOGE(TAG, "Failed to allocate DS18B20_Info for sensor %d", i);
                alloc_failed = true;
                break;
            }
            devices[i] = ds18b20_info;

            if (num_devices == 1)
            {
                ESP_LOGI(TAG, "Single-device optimizations enabled");
                ds18b20_init_solo(ds18b20_info, owb);          // only one device on bus
            }
            else
            {
                ds18b20_init(ds18b20_info, owb, device_rom_codes[i]); // associate with bus and device
            }
            ds18b20_use_crc(ds18b20_info, true);           // enable CRC check on all reads
            ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
        }

        if (alloc_failed) {
            for (int i = 0; i < num_devices; ++i)
            {
                if (devices[i] != NULL) {
                    ds18b20_free(&devices[i]);
                }
            }
            owb_uninitialize(owb);
            vTaskDelay(pdMS_TO_TICKS(DISCOVERY_RETRY_PERIOD));
            continue;
        }

        // Check for parasitic-powered devices
        bool parasitic_power = false;
        ds18b20_check_for_parasite_power(owb, &parasitic_power);
        if (parasitic_power) {
            ESP_LOGW(TAG, "Parasitic-powered devices detected");
        }

        // In parasitic-power mode, devices cannot indicate when conversions are complete,
        // so waiting for a temperature conversion must be done by waiting a prescribed duration
        owb_use_parasitic_power(owb, parasitic_power);

#ifdef CONFIG_ENABLE_STRONG_PULLUP_GPIO
        // An external pull-up circuit is used to supply extra current to OneWireBus devices
        // during temperature conversions.
        owb_use_strong_pullup_gpio(owb, CONFIG_STRONG_PULLUP_GPIO);
#endif

        // Read temperatures more efficiently by starting conversions on all devices at the same time
        int errors_count[MAX_DEVICES] = {0};
        int sample_count = 0;
        TickType_t last_wake_time = xTaskGetTickCount();

        while (1)
        {
            ds18b20_convert_all(owb);

            // In this application all devices use the same resolution,
            // so use the first device to determine the delay
            ds18b20_wait_for_conversion(devices[0]);

            DS18B20_ERROR errors[MAX_DEVICES] = {0};
            lock_take();
            for (int i = 0; i < num_devices; ++i)
            {
                float new_temp = 0.0f;
                errors[i] = ds18b20_read_temp(devices[i], &new_temp);

                if (errors[i] == DS18B20_OK &&
                    new_temp >= DS18B20_TEMP_MIN_C &&
                    new_temp <= DS18B20_TEMP_MAX_C)
                {
                    readings[i] = new_temp;
                }
                else
                {
                    errors[i] = DS18B20_ERROR_DEVICE;
                }
            }

            ESP_LOGI(TAG, "Temperature readings sample %d", ++sample_count);
            for (int i = 0; i < num_devices; ++i)
            {
                if (errors[i] != DS18B20_OK)
                {
                    ++errors_count[i];
                }

                ESP_LOGI(TAG, "Sensor %d: %.1f C (%d errors)", i, readings[i], errors_count[i]);
            }
            lock_give();
            vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
        }
    }
}

float esp_ds18b20_get_readings(int index){
    if (index < 0 || index >= num_devices) {
        return -100.0f; // Invalid index, return an out-of-range value
    }

    lock_take();
    float value = readings[index];
    lock_give();
    return value;
}