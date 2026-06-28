#include "../include/nvs.h"

// NVS
// ****************************************************************
void nvs_init(void)
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

// write IMU calibration offsets to NVS
esp_err_t NVS_write_imu_calibration_offsets(imu_calibration_offsets_t *offsets)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open NVS handle
    ESP_LOGI(NVS_TAG, "\nOpening Non-Volatile Storage (NVS) handle...");
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(NVS_TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    // Write blob
    ESP_LOGI(NVS_TAG, "write IMU calibration offsets to NVS...");
    err = nvs_set_blob(my_handle, IMU_OFFSETS_NVS_KEY, offsets, sizeof(imu_calibration_offsets_t));
    if (err != ESP_OK)
    {
        ESP_LOGE(NVS_TAG, "Failed to write IMU calibration offsets to NVS!");
        nvs_close(my_handle);
        return err;
    }

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(NVS_TAG, "Failed to commit data");
    }

    nvs_close(my_handle);
    return err;
}

// read IMU calibration offsets from NVS
esp_err_t NVS_read_imu_calibration_offsets(imu_calibration_offsets_t *offsets)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    // initialize the required size to store the offsets
    size_t required_size = sizeof(imu_calibration_offsets_t);
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK)
        return err;

    // 1. Read test data blob
    ESP_LOGI(NVS_TAG, "IMU calibration offsets from blob:");
    err = nvs_get_blob(my_handle, IMU_OFFSETS_NVS_KEY, offsets, &required_size);
    ESP_LOGI(NVS_TAG, "required_size = %d", required_size);
    ESP_LOGI(NVS_TAG, "err=%d\n", err);
    if (err == ESP_OK)
    {
        // x
        ESP_LOGI(NVS_TAG, "accel_offsets_x = %.4f", offsets->oAx);
        ESP_LOGI(NVS_TAG, "accel_offsets_y = %.4f", offsets->oAy);
        ESP_LOGI(NVS_TAG, "accel_offsets_z = %.4f", offsets->oAz);
        ESP_LOGI(NVS_TAG, "gyro_offsets_x = %.4f", offsets->oGx);
        ESP_LOGI(NVS_TAG, "gyro_offsets_y = %.4f", offsets->oGy);
        ESP_LOGI(NVS_TAG, "gyro_offsets_z = %.4f", offsets->oGz);
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGW(NVS_TAG, "Test data not found!");
    }
    nvs_close(my_handle);
    return ESP_OK;
}

// ****************************************************************
