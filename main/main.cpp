/**
 * esp32_iot/main.cpp
 *
 * includes
 * defines
 * typedefs
 * mDNS
 * WiFi Station
 * SPIFFS File Server
 * http server
 * I2C Slave
 * app_main
 */

// --- Required includes for RTOS and synchronization ---
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "mdns.h"
#include "lwip/ip4_addr.h"

/*
i2c_slave_v2.c has been modified to disable clock stretching.
This is necessary to prevent the I2C slave from blocking the master when it is not ready
*/

#define I2C_SLAVE_SCL_IO (gpio_num_t)9
#define I2C_SLAVE_SDA_IO (gpio_num_t)8
#define I2C_SLAVE_NUM 0
#define ESP_SLAVE_ADDR 0x22

#define WIFI_SSID_MAXLEN 32 + 1 // max SSID length is 32 characters, +1 for null terminator, see IEEE802.11
#define WIFI_PASS_MAXLEN 64 + 1 // max password length is 64 characters, +1 for null-terminator, see IEEE802.11
#define MDNS_NAME_MAXLEN 32     // max mDNS name length is 32 characters
#define SENSOR_NAME_MAXLEN 32   // max sensor name length is 32 characters
#define SENSOR_NUM 5

#define CMD_MDNS_NAME 0x30
#define CMD_WIFI_SSID 0x31
#define CMD_WIFI_PASS 0x32
#define CMD_WIFI_STA_START 0x33
#define CMD_WIFI_AP_START 0x34

#define CMD_SENSOR_READ 0x40
#define CMD_SENSOR_SETUP 0x41

#define CMD_SEND_DATA 0x50

enum units
{
    UNIT_CELSIUS,
    UNIT_FAHRENHEIT,
    UNIT_KELVIN,
    UNIT_CM,
    UNIT_PERCENT,
    UNIT_NONE
};

typedef struct
{
    char wifi_ssid[WIFI_SSID_MAXLEN];
    char wifi_pass[WIFI_PASS_MAXLEN];
    char mdns_name[MDNS_NAME_MAXLEN];
    uint8_t response_data[4]; // [0]=padding, [1]=door, [2]=fan, [3]=light
    bool wifi_started;
    SemaphoreHandle_t ret_cmd_mutex;
    SemaphoreHandle_t sensor_mutex;
    SemaphoreHandle_t name_mutex;
    httpd_handle_t http_server;
    float sensor_data[SENSOR_NUM];
    char sensor_name[SENSOR_NUM][SENSOR_NAME_MAXLEN];
} i2c_slave_context_t;

i2c_slave_context_t context = {
    .wifi_ssid = "",
    .wifi_pass = "",
    .mdns_name = "iot",
    .response_data = {0x69, 0, 0, 0},
    .wifi_started = false,
    .ret_cmd_mutex = NULL,
    .sensor_mutex = NULL,
    .name_mutex = NULL,
    .http_server = NULL,
    .sensor_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .sensor_name = {
        "Temperature",
        "Humidity",
        "Light",
        "Ultrasound",
        "Magnetic"}};

static void log_runtime_ready(void)
{
    ESP_LOGI("STATUS", "ESP32 IoT firmware booted");
    ESP_LOGI("STATUS", "I2C slave ready on SDA=%d SCL=%d addr=0x%02X", I2C_SLAVE_SDA_IO, I2C_SLAVE_SCL_IO, ESP_SLAVE_ADDR);
    ESP_LOGI("STATUS", "Send I2C command 0x50 then read 4 bytes to verify link");
    ESP_LOGI("STATUS", "Available commands: MDNS=0x30 SSID=0x31 PASS=0x32 STA=0x33 AP=0x34 SENSOR=0x40 SEND_DATA=0x50");
}

static void log_i2c_bytes(const char *label, const uint8_t *data, size_t len)
{
    char hex_buffer[3 * 16 + 1] = {0};
    size_t shown = (len < 16) ? len : 16;
    size_t offset = 0;

    for (size_t index = 0; index < shown && offset < sizeof(hex_buffer); ++index)
    {
        offset += snprintf(&hex_buffer[offset], sizeof(hex_buffer) - offset, "%02X%s",
                           data[index], (index + 1 < shown) ? " " : "");
    }

    ESP_LOGI("I2C", "%s len=%u bytes=%s%s", label, (unsigned)len, hex_buffer,
             (len > shown) ? " ..." : "");
}

static bool should_log_i2c_command(uint8_t cmd)
{
    switch (cmd)
    {
    case CMD_WIFI_SSID:
    case CMD_WIFI_PASS:
    case CMD_WIFI_STA_START:
    case CMD_WIFI_AP_START:
    case CMD_MDNS_NAME:
    case CMD_SEND_DATA:
        return true;
    default:
        return false;
    }
}

static esp_err_t queue_response_data(void)
{
    uint8_t snapshot[sizeof(context.response_data)];

    xSemaphoreTake(context.ret_cmd_mutex, portMAX_DELAY);
    memcpy(snapshot, context.response_data, sizeof(snapshot));
    xSemaphoreGive(context.ret_cmd_mutex);
    log_i2c_bytes("Queueing TX response", snapshot, sizeof(snapshot));

    int written = i2c_slave_write_buffer((i2c_port_t)I2C_SLAVE_NUM, snapshot, sizeof(snapshot), 0);
    if (written != (int)sizeof(snapshot))
    {
        ESP_LOGW("I2C", "TX queue write incomplete: wrote=%d expected=%u", written, (unsigned)sizeof(snapshot));
    }
    return (written == (int)sizeof(snapshot)) ? ESP_OK : ESP_ERR_TIMEOUT;
}

// ===== mDNS ======

static void initialise_mdns(const char *hostname = "iot")
{
    // Initialize mDNS
    ESP_ERROR_CHECK(mdns_init());

    // Set mDNS hostname
    ESP_ERROR_CHECK(mdns_hostname_set(hostname));
    ESP_LOGI("mDNS", "mDNS hostname set to: [%s]", hostname);

    // Set default mDNS instance name
    ESP_ERROR_CHECK(mdns_instance_name_set("ESP32 IoT Device"));
}

// ===== WiFi Station =====

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "wifi station";
static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < 20) // Retry up to 20 times
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    ESP_LOGI("STATUS", "Starting WiFi station mode for SSID: %s", context.wifi_ssid);
    s_wifi_event_group = xEventGroupCreate();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {};
    strlcpy((char *)wifi_config.sta.ssid, context.wifi_ssid, WIFI_SSID_MAXLEN);
    strlcpy((char *)wifi_config.sta.password, context.wifi_pass, WIFI_PASS_MAXLEN);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap");
        ESP_LOGI("STATUS", "WiFi station is working. Check the IP above and open the web page from that address.");
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect");
        ESP_LOGW("STATUS", "WiFi station failed. Recheck SSID/password sent over I2C.");
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// Wi-Fi init for AP mode (ESP32 be Wi-Fi AP, devices connect to ESP32)
void wifi_init_ap(void)
{
    ESP_LOGI("STATUS", "Starting WiFi access point mode");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {};
    strlcpy((char *)ap_config.ap.ssid, context.wifi_ssid, sizeof(ap_config.ap.ssid));
    ap_config.ap.ssid_len = strlen(context.wifi_ssid);
    strlcpy((char *)ap_config.ap.password, context.wifi_pass, sizeof(ap_config.ap.password));
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    if (strlen(context.wifi_pass) == 0)
    {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("wifi", "WiFi AP started. SSID:%s password:%s", context.wifi_ssid, context.wifi_pass);
    ESP_LOGI("STATUS", "AP mode is working. Connect a phone/PC to SSID '%s' and then open 192.168.4.1", context.wifi_ssid);
    ESP_LOGI("STATUS", "HTTP endpoints: /status /sensor /name /set_cmd");
}

// ===== SPIFFS File Server =====
// ===== Set Command Handler =====
static esp_err_t set_cmd_handler(httpd_req_t *req)
{
    ESP_LOGI("HTTP", "Received set_cmd request");
    char buf[32];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[ret] = 0;
    int door = -1, fan = -1, light = -1;
    sscanf(buf, "door=%d&fan=%d&light=%d", &door, &fan, &light);
    if (door < 0 || door > 1 || fan < 0 || fan > 255 || light < 0 || light > 255)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid value");
        return ESP_FAIL;
    }

    xSemaphoreTake(context.ret_cmd_mutex, portMAX_DELAY);
    // context.response_data[0] = 0x69; // Padding/garbage, no need change
    context.response_data[1] = (uint8_t)door;
    context.response_data[2] = (uint8_t)fan;
    context.response_data[3] = (uint8_t)light;
    xSemaphoreGive(context.ret_cmd_mutex);

    if (queue_response_data() != ESP_OK)
    {
        ESP_LOGW("I2C", "Failed to queue response data for master read");
    }
    else
    {
        ESP_LOGI("STATUS", "HTTP updated response bytes successfully");
    }

    ESP_LOGI("HTTP", "Updated: door=%d, fan=%d, light=%d", door, fan, light);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// ===== Status GET Handler =====
static esp_err_t status_handler(httpd_req_t *req)
{
    ESP_LOGI("HTTP", "Received status request");

    // Get current status
    uint8_t door_state, fan_level, light_level;
    xSemaphoreTake(context.ret_cmd_mutex, portMAX_DELAY);
    door_state = context.response_data[1];
    fan_level = context.response_data[2];
    light_level = context.response_data[3];
    xSemaphoreGive(context.ret_cmd_mutex);

    // Compose JSON response with door_state, fan_level, light_level
    char response[256];
    snprintf(response, sizeof(response),
             "{\"door_state\":%d,\"fan_level\":%d,\"light_level\":%d}",
             door_state, fan_level, light_level);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, response);
    return ESP_OK;
}
static esp_err_t sensor_handler(httpd_req_t *req)
{
    ESP_LOGI("HTTP", "Received sensor request");

    // Get current status
    float sensor_snapshot[SENSOR_NUM];
    xSemaphoreTake(context.sensor_mutex, portMAX_DELAY);
    memcpy(sensor_snapshot, context.sensor_data, sizeof(sensor_snapshot));
    xSemaphoreGive(context.sensor_mutex);

    uint8_t door_state, fan_level, light_level;
    xSemaphoreTake(context.ret_cmd_mutex, portMAX_DELAY);
    door_state = context.response_data[1];
    fan_level = context.response_data[2];
    light_level = context.response_data[3];
    xSemaphoreGive(context.ret_cmd_mutex);

    // Compose JSON response with sensor_data
    char response[256];
    snprintf(response, sizeof(response),
             "{\"sensor_data\":[%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d]}",
             sensor_snapshot[0], sensor_snapshot[1], sensor_snapshot[2], sensor_snapshot[3],
             sensor_snapshot[4], door_state, fan_level, light_level);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, response);
    return ESP_OK;
}
static esp_err_t name_handler(httpd_req_t *req)
{
    ESP_LOGI("HTTP", "Received name request");

    // Get current status
    char sensor_name[SENSOR_NUM][SENSOR_NAME_MAXLEN] = {0};
    xSemaphoreTake(context.name_mutex, portMAX_DELAY);
    memcpy(sensor_name, context.sensor_name, sizeof(sensor_name));
    xSemaphoreGive(context.name_mutex);

    // Compose JSON response with door_state, fan_level, light_level, and sensor_data
    char response[256];
    snprintf(response, sizeof(response),
             "{\"sensor_name\":[\"%s\",\"%s\",\"%s\",\"%s\",\"%s\"]}",
             sensor_name[0], sensor_name[1], sensor_name[2], sensor_name[3],
             sensor_name[4]);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, response);
    return ESP_OK;
}

static const char *get_mime_type(const char *filename)
{
    if (strstr(filename, ".html"))
        return "text/html";
    if (strstr(filename, ".css"))
        return "text/css";
    if (strstr(filename, ".js"))
        return "application/javascript";
    if (strstr(filename, ".json"))
        return "application/json";
    if (strstr(filename, ".png"))
        return "image/png";
    if (strstr(filename, ".jpg") || strstr(filename, ".jpeg"))
        return "image/jpeg";
    return "text/plain";
}

static esp_err_t file_handler(httpd_req_t *req)
{
    char filepath[64];
    // Handle root path
    if (strcmp(req->uri, "/") == 0)
    {
        snprintf(filepath, sizeof(filepath), "/spiffs/index.html");
    }
    else
    {
        // Limit the length of req->uri to avoid buffer overflow
        snprintf(filepath, sizeof(filepath), "/spiffs%.*s", (int)(sizeof(filepath) - 8 - 1), req->uri);
    }

    FILE *file = fopen(filepath, "r");
    if (!file)
    {
        ESP_LOGE("HTTP", "Failed to open file: %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Set MIME type using actual file path
    httpd_resp_set_type(req, get_mime_type(filepath));

    // Send file in chunks
    char buffer[1024];
    size_t read_bytes;
    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0)
    {
        if (httpd_resp_send_chunk(req, buffer, read_bytes) != ESP_OK)
        {
            fclose(file);
            ESP_LOGE("HTTP", "Failed to send file chunk");
            return ESP_FAIL;
        }
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    fclose(file);

    ESP_LOGI("HTTP", "Served file: %s", filepath);
    return ESP_OK;
}

// ===== HTTP Server Task =====
static void http_server_task(void *arg)
{
    ESP_LOGI("HTTP", "Starting HTTP server task");
    static httpd_handle_t server = NULL;
    if (server)
    {
        httpd_stop(server);
        server = NULL;
    }
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.uri_match_fn = httpd_uri_match_wildcard;
    ESP_ERROR_CHECK(httpd_start(&server, &server_config));

    // Store server handle for SSE
    context.http_server = server;

    // /set_cmd POST handler
    httpd_uri_t set_cmd_uri = {
        .uri = "/set_cmd",
        .method = HTTP_POST,
        .handler = set_cmd_handler,
        .user_ctx = NULL};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &set_cmd_uri));

    // /status GET handler for status polling
    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &status_uri));

    // /sensor GET handler for sensor polling
    httpd_uri_t sensor_uri = {
        .uri = "/sensor",
        .method = HTTP_GET,
        .handler = sensor_handler,
        .user_ctx = NULL};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &sensor_uri));

    // /name GET handler for name polling
    httpd_uri_t name_uri = {
        .uri = "/name",
        .method = HTTP_GET,
        .handler = name_handler,
        .user_ctx = NULL};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &name_uri));

    // Universal file handler for all requests
    httpd_uri_t file_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = file_handler,
        .user_ctx = NULL};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &file_uri));

    ESP_LOGI("HTTP", "HTTP server started with SPIFFS file serving");
    ESP_LOGI("STATUS", "HTTP server ready. Open '/' or poll /status /sensor /name");
    vTaskDelete(NULL);
}

// ===== I2C Slave Task =====

static bool is_i2c_command_byte(uint8_t value)
{
    switch (value)
    {
    case CMD_MDNS_NAME:
    case CMD_WIFI_SSID:
    case CMD_WIFI_PASS:
    case CMD_WIFI_STA_START:
    case CMD_WIFI_AP_START:
    case CMD_SENSOR_READ:
    case CMD_SENSOR_SETUP:
    case CMD_SEND_DATA:
        return true;
    default:
        return false;
    }
}

static bool is_fixed_length_command(uint8_t cmd)
{
    switch (cmd)
    {
    case CMD_WIFI_STA_START:
    case CMD_WIFI_AP_START:
    case CMD_SEND_DATA:
    case CMD_SENSOR_READ:
        return true;
    default:
        return false;
    }
}

static size_t get_fixed_frame_length(uint8_t cmd)
{
    switch (cmd)
    {
    case CMD_WIFI_STA_START:
    case CMD_WIFI_AP_START:
    case CMD_SEND_DATA:
        return 1;
    case CMD_SENSOR_READ:
        return 6;
    default:
        return 0;
    }
}

static void handle_i2c_command_frame(uint8_t cmd, const uint8_t *payload, size_t payload_len)
{
    uint8_t frame_buffer[150];
    size_t frame_len = payload_len + 1;
    frame_buffer[0] = cmd;
    if (payload_len > 0)
    {
        memcpy(&frame_buffer[1], payload, payload_len);
    }

    bool log_command = should_log_i2c_command(cmd);
    if (log_command)
    {
        ESP_LOGI("I2C", "Processing RX cmd 0x%02X of length %u", cmd, (unsigned)frame_len);
        log_i2c_bytes("RX frame", frame_buffer, frame_len);
    }

    switch (cmd)
    {
    case CMD_MDNS_NAME:
    {
        if (payload_len > 0)
        {
            size_t mdns_len = payload_len;
            if (mdns_len > MDNS_NAME_MAXLEN)
                mdns_len = MDNS_NAME_MAXLEN;
            memcpy(context.mdns_name, payload, mdns_len);
            context.mdns_name[mdns_len] = '\0';
            mdns_hostname_set(context.mdns_name);
            ESP_LOGI("I2C", "mDNS name set to: %s", context.mdns_name);
            ESP_LOGI("STATUS", "Device hostname should answer as %s.local", context.mdns_name);
        }
        break;
    }
    case CMD_WIFI_SSID:
    {
        if (context.wifi_started)
        {
            ESP_LOGW("I2C", "Cannot set WiFi SSID while WiFi is started");
            break;
        }
        if (payload_len > 0)
        {
            size_t ssid_len = payload_len;
            if (ssid_len > WIFI_SSID_MAXLEN - 1)
                ssid_len = WIFI_SSID_MAXLEN - 1;
            memcpy(context.wifi_ssid, payload, ssid_len);
            context.wifi_ssid[ssid_len] = '\0';
            ESP_LOGI("I2C", "Wifi SSID set to %s", context.wifi_ssid);
            ESP_LOGI("STATUS", "Stored WiFi SSID length=%u", (unsigned)ssid_len);
        }
        break;
    }
    case CMD_WIFI_PASS:
    {
        if (context.wifi_started)
        {
            ESP_LOGW("I2C", "Cannot set WiFi password while WiFi is started");
            break;
        }
        if (payload_len > 0)
        {
            size_t pass_len = payload_len;
            if (pass_len > WIFI_PASS_MAXLEN - 1)
                pass_len = WIFI_PASS_MAXLEN - 1;
            memcpy(context.wifi_pass, payload, pass_len);
            context.wifi_pass[pass_len] = '\0';
            ESP_LOGI("I2C", "Wifi password updated");
            ESP_LOGI("STATUS", "Stored WiFi password length=%u", (unsigned)pass_len);
        }
        break;
    }
    case CMD_WIFI_STA_START:
    {
        if (context.wifi_started)
        {
            ESP_LOGW("I2C", "WiFi already started");
            break;
        }
        wifi_init_sta();
        context.wifi_started = true;
        ESP_LOGI("STATUS", "I2C requested WiFi STA start");

        xTaskCreate(http_server_task, "http_server_task", 4096, NULL, 6, NULL);
        break;
    }
    case CMD_WIFI_AP_START:
    {
        if (context.wifi_started)
        {
            ESP_LOGW("I2C", "WiFi already started");
            break;
        }
        wifi_init_ap();
        context.wifi_started = true;
        ESP_LOGI("STATUS", "I2C requested WiFi AP start");

        xTaskCreate(http_server_task, "http_server_task", 4096, NULL, 6, NULL);
        break;
    }
    case CMD_SENSOR_READ:
    {
        if (payload_len == 5 && payload[0] < SENSOR_NUM)
        {
            context.sensor_data[payload[0]] = *((float *)&payload[1]);
        }
        else
        {
            ESP_LOGW("I2C", "Invalid sensor frame payload_len=%u sensor_index=%u", (unsigned)payload_len, (payload_len > 0) ? payload[0] : 0xFF);
        }
        break;
    }
    case CMD_SENSOR_SETUP:
    {
        if (payload_len > 0)
            ESP_LOGI("I2C", "Sensor data updated");
        break;
    }
    case CMD_SEND_DATA:
    {
        if (queue_response_data() != ESP_OK)
        {
            ESP_LOGW("I2C", "Failed to queue response data for CMD_SEND_DATA");
        }
        else
        {
            ESP_LOGI("STATUS", "I2C read check OK. Response bytes queued: [%u, %u, %u, %u]",
                     context.response_data[0], context.response_data[1], context.response_data[2], context.response_data[3]);
        }
        break;
    }
    default:
    {
        ESP_LOGW("I2C", "Unknown command 0x%02X", cmd);
        break;
    }
    }
}

static void i2c_slave_task(void *arg)
{
    uint8_t cmd_data[150];

    while (true)
    {
        int cmd_len = i2c_slave_read_buffer((i2c_port_t)I2C_SLAVE_NUM, cmd_data, sizeof(cmd_data), pdMS_TO_TICKS(10));
        if (cmd_len > 0)
        {
            size_t offset = 0;

            while (offset < (size_t)cmd_len)
            {
                if (!is_i2c_command_byte(cmd_data[offset]))
                {
                    ESP_LOGW("I2C", "Discarding unexpected byte 0x%02X at offset %u", cmd_data[offset], (unsigned)offset);
                    ++offset;
                    continue;
                }

                uint8_t cmd = cmd_data[offset];
                size_t remaining = (size_t)cmd_len - offset;

                if (is_fixed_length_command(cmd))
                {
                    size_t frame_len = get_fixed_frame_length(cmd);
                    if (remaining < frame_len)
                    {
                        ESP_LOGW("I2C", "Incomplete fixed frame for cmd 0x%02X at offset %u", cmd, (unsigned)offset);
                        break;
                    }
                    handle_i2c_command_frame(cmd, &cmd_data[offset + 1], frame_len - 1);
                    offset += frame_len;
                    continue;
                }

                if (remaining >= 2)
                {
                    size_t payload_len = cmd_data[offset + 1];
                    size_t frame_len = payload_len + 2;
                    if (frame_len <= remaining)
                    {
                        handle_i2c_command_frame(cmd, &cmd_data[offset + 2], payload_len);
                        offset += frame_len;
                        continue;
                    }
                }

                // Backward-compatible fallback: a single legacy variable-length frame occupying the rest of the burst.
                if (offset == 0)
                {
                    if (should_log_i2c_command(cmd))
                    {
                        ESP_LOGW("I2C", "Using legacy variable-length parsing for cmd 0x%02X; prefer [cmd][len][payload] framing", cmd);
                    }
                    handle_i2c_command_frame(cmd, &cmd_data[offset + 1], remaining - 1);
                    break;
                }

                ESP_LOGW("I2C", "Incomplete variable-length frame for cmd 0x%02X at offset %u", cmd, (unsigned)offset);
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

// debug print function, prints WiFi and mDNS info every second
[[maybe_unused]] static void print_wifi_info_task(void *arg)
{
    while (true)
    {
        printf("SSID: %s\n", context.wifi_ssid);
        printf("Password: %s\n", context.wifi_pass);
        printf("mDNS name: %s\n", context.mdns_name);

        // WiFi status
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
        {
            printf("WiFi Status: Connected to %s\n", ap_info.ssid);
        }
        else
        {
            printf("WiFi Status: Not connected\n");
        }

        // IP address
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        esp_netif_ip_info_t ip_info;
        if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0)
        {
            printf("IP Address: %d.%d.%d.%d\n",
                   ip4_addr1_16(&ip_info.ip),
                   ip4_addr2_16(&ip_info.ip),
                   ip4_addr3_16(&ip_info.ip),
                   ip4_addr4_16(&ip_info.ip));
        }
        else
        {
            printf("IP Address: Not assigned\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize netif and event loop ONCE
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    initialise_mdns();

    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    // Initialize SPIFFS
    esp_vfs_spiffs_conf_t spiffs_conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 5,
        .format_if_mount_failed = true};

    esp_err_t ret = esp_vfs_spiffs_register(&spiffs_conf);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE("SPIFFS", "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE("SPIFFS", "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE("SPIFFS", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
    }
    else
    {
        ESP_LOGI("SPIFFS", "SPIFFS mounted successfully");

        // Check SPIFFS usage
        size_t total = 0, used = 0;
        ret = esp_spiffs_info("storage", &total, &used);
        if (ret == ESP_OK)
        {
            ESP_LOGI("SPIFFS", "Partition size: total: %d, used: %d", total, used);
        }
    }

    // I2C slave config
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_SLAVE;
    conf.sda_io_num = I2C_SLAVE_SDA_IO;
    conf.scl_io_num = I2C_SLAVE_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.slave.addr_10bit_en = 0;
    conf.slave.slave_addr = ESP_SLAVE_ADDR;
    conf.slave.maximum_speed = 400000;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    context.ret_cmd_mutex = xSemaphoreCreateMutex();
    context.sensor_mutex = xSemaphoreCreateMutex();
    context.name_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_param_config((i2c_port_t)I2C_SLAVE_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install((i2c_port_t)I2C_SLAVE_NUM, conf.mode, 150, 16, 0));
    ESP_LOGI("STATUS", "I2C driver installed with RX buffer=150 TX buffer=16");

    if (queue_response_data() != ESP_OK)
    {
        ESP_LOGW("I2C", "Initial response data was not queued");
    }

    ESP_LOGI("I2C", "I2C slave initialized on SCL %d, SDA %d", I2C_SLAVE_SCL_IO, I2C_SLAVE_SDA_IO);
    log_runtime_ready();

    xTaskCreate(i2c_slave_task, "i2c_slave_task", 4 * 1024, &context, 10, NULL);
    // xTaskCreate(print_wifi_info_task, "print_wifi_info_task", 4096, &context, 5, NULL);
}