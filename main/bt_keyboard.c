/*
* bt_keyboard.c - BLE HID Keyboard using esp_hidh
*/

#include "bt_keyboard.h"
#include "my_console_io.h"
#include "esp_log.h"
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_bt.h"
#include "esp_hidh.h"
#include "esp_hid_common.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "host/ble_store.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h" // Added for timer

void ble_store_config_init(void);

static const char *TAG = "bt_kbd";

// NVS keys
#define NVS_NAMESPACE   "bt_kbd"
#define NVS_KEY_ADDR    "addr"
#define NVS_KEY_TYPE    "type"

// State
static uint8_t s_own_addr_type;
static int s_is_scanning = 0;
static int s_connected = 0;
static int s_encrypted = 0;

// Reconnect Logic State
static int s_reconnect_mode = 0;        
static int s_scan_on_connect_fail = 0;  

// Target keyboard
static uint8_t s_target_bda[6];
static uint8_t s_target_type;
static int s_have_target = 0;
static volatile int s_connect_pending = 0; // Marked volatile for safety

// Connection task & Timer
static TaskHandle_t s_connect_task = NULL;
static TimerHandle_t s_boot_timer = NULL; // Timer handle

// Key processing state
static uint8_t s_last_keys[6] = {0};

// Forward Declarations
static void start_background_scan(void);
static void start_general_scan(void);
static void request_connection(void);

// --- KEY MAP ---
static const char HID_MAP[] = {
    0,0,0,0,'a','b','c','d','e','f','g','h','i','j','k','l','m','n',
    'o','p','q','r','s','t','u','v','w','x','y','z','1','2','3','4',
    '5','6','7','8','9','0','\n',27,'\b','\t',' ','-','=','[',']','\\',
    0,';','\'','`',',','.','/'
};
static const char HID_MAP_SHIFT[] = {
    0,0,0,0,'A','B','C','D','E','F','G','H','I','J','K','L','M','N',
    'O','P','Q','R','S','T','U','V','W','X','Y','Z','!','@','#','$',
    '%','^','&','*','(',')','\n',27,'\b','\t',' ','_','+','{','}','|',
    0,':','"','~','<','>','?'
};

// ============ NVS Functions ============

static void save_target_to_nvs(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, NVS_KEY_ADDR, s_target_bda, 6);
        nvs_set_u8(h, NVS_KEY_TYPE, s_target_type);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Saved keyboard Identity Address to NVS");
    }
}

static void load_target_from_nvs(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_OK) {
        size_t len = 6;
        esp_err_t e1 = nvs_get_blob(h, NVS_KEY_ADDR, s_target_bda, &len);
        esp_err_t e2 = nvs_get_u8(h, NVS_KEY_TYPE, &s_target_type);
        
        if (e1 == ESP_OK && e2 == ESP_OK) {
            s_have_target = 1;
            ESP_LOGI(TAG, "Loaded saved keyboard: %02x:%02x:%02x:%02x:%02x:%02x type=%d",
                    s_target_bda[5], s_target_bda[4], s_target_bda[3],
                    s_target_bda[2], s_target_bda[1], s_target_bda[0], s_target_type);
        }
        nvs_close(h);
    }
}

// ============ GAP Event Listener ============

static struct ble_gap_event_listener s_gap_listener;

static int gap_event_listener(struct ble_gap_event *event, void *arg) {
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "Connected!");

            // 3. TURBO MODE: Request fastest possible connection interval (7.5ms - 15ms)
            // This makes the service discovery (the "spammy" part) finish much faster.
            struct ble_gap_upd_params params = {0};
            params.itvl_min = 6;  // 6 * 1.25ms = 7.5ms
            params.itvl_max = 12; // 12 * 1.25ms = 15ms
            params.latency = 0;
            params.supervision_timeout = 100; // 1 second
            
            rc = ble_gap_update_params(event->connect.conn_handle, &params);
            if (rc != 0) {
                 ESP_LOGW(TAG, "Failed to update params: %d", rc);
            }

            // Initiate security
            ble_gap_security_initiate(event->connect.conn_handle);

            // Save ID address logic...
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc == 0) {
                // (Existing logic to save address to NVS...)
                memcpy(s_target_bda, desc.peer_id_addr.val, 6);
                s_target_type = desc.peer_id_addr.type;
                s_have_target = 1;
                save_target_to_nvs();
            }
        }
        break;
                
    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            ESP_LOGI(TAG, ">>> ENCRYPTED <<<");
            s_encrypted = 1;
        } else {
            ESP_LOGE(TAG, "Encryption failed: %d", event->enc_change.status);
        }
        break;
        
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE Disconnected");
        s_encrypted = 0;
        s_connected = 0;
        // Optional: Trigger background scan immediately on disconnect?
        // start_background_scan(); 
        break;
        
    case BLE_GAP_EVENT_REPEAT_PAIRING:
        ESP_LOGI(TAG, "Repeat pairing - clearing old bond");
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    
    return 0;
}

// ============ Key Processing ============

// HID keycodes for special keys
#define HID_KEY_ESC         0x29
#define HID_KEY_F1          0x3A
#define HID_KEY_F12         0x45
#define HID_KEY_INSERT      0x49
#define HID_KEY_HOME        0x4A
#define HID_KEY_PAGEUP      0x4B
#define HID_KEY_DELETE      0x4C
#define HID_KEY_END         0x4D
#define HID_KEY_PAGEDOWN    0x4E
#define HID_KEY_RIGHT       0x4F
#define HID_KEY_LEFT        0x50
#define HID_KEY_DOWN        0x51
#define HID_KEY_UP          0x52

// Send ANSI escape sequence for special keys
// Returns 1 if key was handled, 0 otherwise
static int handle_special_key(uint8_t key) {
    const char *seq = NULL;

    switch (key) {
    // Arrow keys: ESC [ A/B/C/D
    case HID_KEY_UP:    seq = "\x1b[A"; break;
    case HID_KEY_DOWN:  seq = "\x1b[B"; break;
    case HID_KEY_RIGHT: seq = "\x1b[C"; break;
    case HID_KEY_LEFT:  seq = "\x1b[D"; break;

    // Navigation keys
    case HID_KEY_HOME:     seq = "\x1b[H"; break;
    case HID_KEY_END:      seq = "\x1b[F"; break;
    case HID_KEY_INSERT:   seq = "\x1b[2~"; break;
    case HID_KEY_DELETE:   seq = "\x1b[3~"; break;
    case HID_KEY_PAGEUP:   seq = "\x1b[5~"; break;
    case HID_KEY_PAGEDOWN: seq = "\x1b[6~"; break;

    // F1-F12 keys (xterm-style sequences)
    case HID_KEY_F1:  seq = "\x1bOP"; break;
    case HID_KEY_F1+1: seq = "\x1bOQ"; break;
    case HID_KEY_F1+2: seq = "\x1bOR"; break;
    case HID_KEY_F1+3: seq = "\x1bOS"; break;
    case HID_KEY_F1+4: seq = "\x1b[15~"; break;
    case HID_KEY_F1+5: seq = "\x1b[17~"; break;
    case HID_KEY_F1+6: seq = "\x1b[18~"; break;
    case HID_KEY_F1+7: seq = "\x1b[19~"; break;
    case HID_KEY_F1+8: seq = "\x1b[20~"; break;
    case HID_KEY_F1+9: seq = "\x1b[21~"; break;
    case HID_KEY_F1+10: seq = "\x1b[23~"; break;
    case HID_KEY_F1+11: seq = "\x1b[24~"; break;

    default:
        return 0;  // Not a special key
    }

    // Send each character of the sequence
    while (*seq) {
        my_console_bt_receive(*seq++);
    }
    return 1;
}

static void process_key_report(const uint8_t *data, size_t len) {
    if (len < 2) return;

    uint8_t mod = data[0];
    int key_start = (len == 8) ? 2 : 1;

    int shift = (mod & 0x22);
    int ctrl = (mod & 0x11);

    for (int i = key_start; i < len && i < key_start + 6; i++) {
        uint8_t key = data[i];
        if (key == 0) continue;

        int already = 0;
        for (int j = 0; j < 6; j++) {
            if (s_last_keys[j] == key) already = 1;
        }

        if (!already) {
            // Try special keys first (arrows, F-keys, etc.)
            if (handle_special_key(key)) {
                continue;  // Handled as special key
            }

            // Regular key from HID_MAP
            if (key < sizeof(HID_MAP)) {
                char c = shift ? HID_MAP_SHIFT[key] : HID_MAP[key];
                if (ctrl && c >= 'a' && c <= 'z') c = c - 'a' + 1;
                if (c) my_console_bt_receive(c);
            }
        }
    }

    for (int i = 0; i < 6; i++) {
        s_last_keys[i] = (key_start + i < len) ? data[key_start + i] : 0;
    }
}

// ============ HIDH Callback ============

static void hidh_callback(void *arg, esp_event_base_t base, int32_t id, void *data) {
    esp_hidh_event_data_t *p = (esp_hidh_event_data_t *)data;
    
    switch ((esp_hidh_event_t)id) {
    case ESP_HIDH_OPEN_EVENT:
        if (p->open.status == ESP_OK) {
            ESP_LOGI(TAG, ">>> KEYBOARD CONNECTED (HID) <<<");
            s_connected = 1;
        } else {
            ESP_LOGE(TAG, "HID open failed: %d", p->open.status);
            s_connected = 0;
        }
        break;
        
    case ESP_HIDH_CLOSE_EVENT:
        ESP_LOGI(TAG, "Keyboard disconnected (HID Close)");
        s_connected = 0;
        // Try to reconnect directly
        request_connection();
        break;
        
    case ESP_HIDH_INPUT_EVENT:
        if (p->input.usage == ESP_HID_USAGE_KEYBOARD) {
            process_key_report(p->input.data, p->input.length);
        }
        break;
        
    default:
        break;
    }
}

// ============ Connection Task ============

static void connect_task(void *arg) {
    ESP_LOGI(TAG, "Connect task started");
    
    while (1) {
        // Wait for notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Debug: We woke up.
        // If this prints, the timer successfully triggered the task.
        ESP_LOGI(TAG, "Connect task triggered (pending=%d target=%d)", s_connect_pending, s_have_target);
        
        if (!s_connect_pending || s_connected || !s_have_target) {
            s_connect_pending = 0;
            continue;
        }
        
        // 1. STOP SCANNING to avoid Error 15 (BLE_HS_EBUSY)
        if (s_is_scanning) {
            ble_gap_disc_cancel();
            s_is_scanning = 0;
            // Short delay to allow Link Layer to process the cancel
            vTaskDelay(pdMS_TO_TICKS(100)); 
        }

        ESP_LOGI(TAG, ">>> Connecting to %02x:%02x:%02x:%02x:%02x:%02x <<<",
                s_target_bda[5], s_target_bda[4], s_target_bda[3],
                s_target_bda[2], s_target_bda[1], s_target_bda[0]);
        
        s_connect_pending = 0;
        s_encrypted = 0;
        
        // This BLOCKS until connection + GATT discovery completes (or fails)
        esp_hidh_dev_t *dev = esp_hidh_dev_open(s_target_bda, ESP_HID_TRANSPORT_BLE, s_target_type);
        
        if (dev != NULL) {
            ESP_LOGI(TAG, "Connection successful");
            s_scan_on_connect_fail = 0; // Success, clear flag
        } else {
            ESP_LOGW(TAG, "Connection failed.");
            
            // Check logic: Did user ask for a scan if this failed?
            if (s_scan_on_connect_fail) {
                ESP_LOGW(TAG, "Reconnect failed, falling back to general scan...");
                s_scan_on_connect_fail = 0;
                start_general_scan();
            } 
            // Fall back to background scan (passive listener)
            else if (!s_connected) {
                ESP_LOGI(TAG, "Starting background scan...");
                start_background_scan();
            }
        }
    }
}

static void request_connection(void) {
    if (s_connect_task && !s_connected && !s_connect_pending && s_have_target) {
        s_connect_pending = 1;
        xTaskNotifyGive(s_connect_task);
        ESP_LOGI(TAG, "Connection task notified");
    }
}

// ============ Scan Callback ============

static int scan_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_DISC) {
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) != 0) {
            return 0;
        }
        
        int is_hid = 0;
        if (fields.appearance_is_present && fields.appearance == 0x03C1) is_hid = 1;
        for (int i = 0; i < fields.num_uuids16; i++) {
            if (ble_uuid_u16(&fields.uuids16[i].u) == 0x1812) is_hid = 1;
        }
        if (!is_hid) return 0;
        
        int match = 0;
        if (s_have_target) {
            if (memcmp(event->disc.addr.val, s_target_bda, 6) == 0) {
                match = 1;
            } 
        }

        if (s_reconnect_mode && !match) {
            return 0;
        }
        
        ESP_LOGI(TAG, ">>> Found %s HID Keyboard! <<<", match ? "MATCHING" : "NEW");
        
        ble_gap_disc_cancel();
        s_is_scanning = 0;
        s_reconnect_mode = 0;
        
        memcpy(s_target_bda, event->disc.addr.val, 6);
        s_target_type = event->disc.addr.type;
        s_have_target = 1;
        
        request_connection();
        
    } else if (event->type == BLE_GAP_EVENT_DISC_COMPLETE) {
        s_is_scanning = 0;
        s_reconnect_mode = 0;
        if (!s_connected && s_have_target) {
            start_background_scan();
        }
    }
    
    return 0;
}

// ============ Scanning Functions ============

static void start_background_scan(void) {
    if (s_connected || s_is_scanning || !s_have_target) return;
    
    ESP_LOGI(TAG, "Background scan for saved keyboard...");
    s_is_scanning = 1;
    s_reconnect_mode = 1; 
    
    struct ble_gap_disc_params p = {
        .passive = 1,
        .filter_duplicates = 0,
        .itvl = 0x100,
        .window = 0x50,
    };
    
    if (ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &p, scan_event_cb, NULL) != 0) {
        s_is_scanning = 0;
        s_reconnect_mode = 0;
    }
}

static void start_general_scan(void) {
    if (s_connected) return;

    if (s_is_scanning) {
        ble_gap_disc_cancel();
        s_is_scanning = 0;
    }

    ESP_LOGI(TAG, "Starting general scan for keyboards...");
    s_is_scanning = 1;
    s_reconnect_mode = 0; 
    s_scan_on_connect_fail = 0; 

    struct ble_gap_disc_params p = {
        .passive = 0, 
        .filter_duplicates = 1,
    };
    
    if (ble_gap_disc(s_own_addr_type, 15000, &p, scan_event_cb, NULL) != 0) {
        s_is_scanning = 0;
    }
}

// ============ Public API ============

esp_err_t bt_keyboard_scan_ex(int verbose) {
    if (s_connected) {
        ESP_LOGI(TAG, "Already connected");
        return ESP_OK;
    }
    
    if (s_is_scanning) {
        ble_gap_disc_cancel();
        s_is_scanning = 0;
    }

    if (s_have_target) {
        ESP_LOGI(TAG, "Attempting to reconnect to saved device first...");
        s_scan_on_connect_fail = 1; 
        request_connection();       
        return ESP_OK;
    }

    start_general_scan();
    return ESP_OK;
}

esp_err_t bt_keyboard_scan(void) {
    return bt_keyboard_scan_ex(0);
}

int bt_keyboard_connected(void) {
    return s_connected;
}

esp_err_t bt_keyboard_connect_native(void) {
    if (s_connected) return ESP_OK;
    if (!s_have_target) {
        ESP_LOGW(TAG, "No saved keyboard - use 'btscan'");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Forcing connection attempt...");
    s_scan_on_connect_fail = 0; 
    request_connection();
    return ESP_OK;
}

esp_err_t bt_keyboard_clear_bonds(void) {
    ESP_LOGI(TAG, "Clearing bonds...");
    ble_store_clear();
    
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    s_have_target = 0;
    return ESP_OK;
}

// ============ Timer Callback ============

static void boot_timer_cb(TimerHandle_t xTimer) {
    // CRITICAL: This runs in the FreeRTOS Timer Task (Tmr Svc).
    // The stack is small (default ~2KB). Do NOT use ESP_LOGI or printf here.
    
    if (s_connect_task && !s_connected && s_have_target) {
        s_connect_pending = 1;
        xTaskNotifyGive(s_connect_task);
    }
}

// ============ NimBLE Callbacks ============

static void on_sync(void) {
    ESP_LOGI(TAG, "BLE Synced!");
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    
    if (s_have_target) {
        ESP_LOGI(TAG, "Boot: Have saved keyboard, starting boot timer (1000ms)...");
        if (s_boot_timer) {
            xTimerStart(s_boot_timer, 0);
        }
    }
}

static void on_reset(int reason) {
    ESP_LOGW(TAG, "BLE reset: %d", reason);
}

static void host_task(void *p) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ============ Init ============

esp_err_t bt_keyboard_init(void) {
    // 1. SILENCE LOGS: This drastically speeds up the connection process
    // by not blocking the CPU on UART output during the heavy GATT traffic.
    esp_log_level_set("NimBLE", ESP_LOG_WARN); 
    esp_log_level_set("bt_kbd", ESP_LOG_INFO);

    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    
    load_target_from_nvs();
    
    xTaskCreate(connect_task, "hidh_connect", 8192, NULL, 5, &s_connect_task);

    // 2. FASTER TIMER: Reduce delay from 1000ms to 100ms
    s_boot_timer = xTimerCreate("bt_boot", pdMS_TO_TICKS(100), pdFALSE, NULL, boot_timer_cb);
    
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    nimble_port_init();
    ble_store_config_init(); 
    ble_svc_gap_device_name_set("ESP32-Console");
    
    esp_hidh_config_t hcfg = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
    };
    esp_hidh_init(&hcfg);

    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    
    ble_gap_event_listener_register(&s_gap_listener, gap_event_listener, NULL);
    
    nimble_port_freertos_init(host_task);
    
    return ESP_OK;
}