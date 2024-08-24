// ROUTER
#include "PixhawkArduinoMAVLink.h" //has mavlink.h
#include <string.h>
#include <inttypes.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "nvs_flash.h"
#include <painlessMesh.h> //has tcp related library

/* Mesh WIFI config*/
#define CONFIG_MESH_ROUTER_SSID "Indlab-software 2.4"
#define CONFIG_MESH_ROUTER_PASSWD "happysofts"
#define CONFIG_MESH_AP_PASSWD "12345678"
#define CONFIG_MESH_ROUTE_TABLE_SIZE 50
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};
#define MAX_RETRY 3

#define ENABLE_TCP_DISABLE_Serial2  0

#define BUFFER_SIZE 1024
#define MESH_PACKET_SIZE 128
#define RX_BUFFER_SIZE MESH_PACKET_SIZE
static uint8_t tx_buf[BUFFER_SIZE] = { 0 };
static uint8_t rx_buf[RX_BUFFER_SIZE] = { 0 };

#if (ENABLE_TCP_DISABLE_Serial2 == 1)
  int port_tcp = 5760;
  WiFiServer server(port_tcp);
  WiFiClient client;
#else
  int baudrate = 115200;
   
#endif

int Rtos_delay = 90;

static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;
static esp_netif_t *netif_sta = NULL;
static bool is_running = true;
mesh_addr_t my_address;
int InSerial2PacketSize = 0;
int InTcpPacketSize = 0;

void setup();
void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data);
void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data);

void loop();



void Serial2FlushRx(void) {
    while (Serial2.available() > 0) { Serial2.read(); }
}

void checkClientConnection() {
  #if (ENABLE_TCP_DISABLE_Serial2 == 1) 
    if (!client.connected()) {
        ESP_LOGE("MESH", "Client disconnected, attempting to reconnect...");
        client.stop();
        client = server.available();  // Try to reconnect to the client
        if (client) {
            ESP_LOGE("MESH", "Reconnected to new client");
        } else {
            ESP_LOGE("MESH", "Failed to reconnect client");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
  #endif
}

void esp_mesh_p2p_tx_main(void *arg)
{
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;
    is_running = true;

    while (is_running) {
        // Get the routing table only once or when necessary to reduce overhead
        esp_mesh_get_routing_table(route_table, CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

        int InPacketSize = 0;

        #if (ENABLE_TCP_DISABLE_Serial2 == 1) 
            checkClientConnection();
            InPacketSize = client.available();
        #else
            InPacketSize = Serial2.available();
        #endif

        if (InPacketSize > 0) {
            int len = 0;

            #if (ENABLE_TCP_DISABLE_Serial2 == 1)
                len = client.read(tx_buf, sizeof(tx_buf));
            #else 
                len = Serial2.read(tx_buf, sizeof(tx_buf));
            #endif
            
            // Reduced delay for faster processing
            vTaskDelay(pdMS_TO_TICKS(Rtos_delay));  

            mavlink_message_t msg;
            mavlink_status_t status;
            for (int i = 0; i < len; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, tx_buf[i], &msg, &status)) {
                    uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
                    int send_len = mavlink_msg_to_send_buffer(send_buf, &msg);

                    // Batch send with optimizations
                    int offset = 0;
                    while (offset < send_len) {
                        int packetSize = (send_len - offset) > MESH_PACKET_SIZE ? MESH_PACKET_SIZE : (send_len - offset);
                        uint8_t message[MESH_PACKET_SIZE];
                        memcpy(message, send_buf + offset, packetSize);
                        
                        mesh_data_t data;
                        data.data = message;
                        data.size = packetSize;
                        data.proto = MESH_PROTO_BIN;
                        data.tos = MESH_TOS_P2P;

                        // Iterate over the route table and send data
                        for (int i = 0; i < route_table_size; i++) {
                            if (memcmp(route_table[i].addr, my_address.addr, 6) != 0) {
                                esp_err_t err = esp_mesh_send(&route_table[i], &data, MESH_DATA_P2P, NULL, 0);
                                if (err != ESP_OK) {
                                    ESP_LOGE("MESH", "Error sending to nodes, retrying: %d", err);
                                    err = esp_mesh_send(&route_table[i], &data, MESH_DATA_P2P, NULL, 0);
                                    if (err == ESP_OK) {
                                        ESP_LOGI("MESH", "Data sent successfully after retry");
                                    }
                                } else {
                                    ESP_LOGI("MESH", "Data sent successfully");
                                }
                            }
                        }
                        offset += packetSize;
                    }
                }
            }
        }    
    }

    vTaskDelete(NULL);
}


void esp_mesh_p2p_rx_main(void *arg)
{
    mesh_addr_t from;
    mesh_data_t data;
    int flag = 0;
    is_running = true;

    static uint8_t rx_buf[MESH_PACKET_SIZE];
    static uint8_t reassemblyBuffer[BUFFER_SIZE];
    static int receivedLength = 0;

    while (is_running) {
        data.data = rx_buf;
        data.size = sizeof(rx_buf);
        esp_err_t err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        if (err != ESP_OK) {
            ESP_LOGE("MESH", "Error receiving data: %d", err);
            continue;
        } 

        ESP_LOGI("MESH", "Data received from node successfully");

        if (receivedLength + data.size <= BUFFER_SIZE) {
            memcpy(reassemblyBuffer + receivedLength, data.data, data.size);
            receivedLength += data.size;

            mavlink_message_t message;
            mavlink_status_t status;
            int bytesParsed = 0;

            for (int i = 0; i < receivedLength; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, reassemblyBuffer[i], &message, &status)) {
                    uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
                    int send_len = mavlink_msg_to_send_buffer(send_buf, &message);

                    #if (ENABLE_TCP_DISABLE_Serial2 == 1)
                        checkClientConnection();
                        if (client.connected()) {
                            client.write(send_buf, send_len);
                        } else {
                            ESP_LOGE("MESH", "Client disconnected, unable to send data");
                        }
                    #else
                        Serial2.write(send_buf, send_len);
                        Serial2FlushRx();
                    #endif

                    bytesParsed = i + 1;
                }
            }

            // Shift remaining bytes in the buffer
            if (bytesParsed > 0) {
                memmove(reassemblyBuffer, reassemblyBuffer + bytesParsed, receivedLength - bytesParsed);
                receivedLength -= bytesParsed;
            }
        } else {
            ESP_LOGW("MESH", "Buffer overflow detected. Message too large.");
            receivedLength = 0;
        }
    }

    vTaskDelete(NULL);
}



esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 8000, NULL, 5, NULL);
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 8000, NULL, 5, NULL);
    }
    return ESP_OK;
}


void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI("MESH", "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));
}



void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint16_t last_layer = 0;

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        ESP_LOGI("MESH", "<MESH_EVENT_MESH_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI("MESH", "<MESH_EVENT_STOPPED>");
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        ESP_LOGI("MESH", "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
                 child_connected->aid,
                 MAC2STR(child_connected->mac));
        esp_mesh_comm_p2p_start();
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI("MESH", "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 child_disconnected->aid,
                 MAC2STR(child_disconnected->mac));
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW("MESH", "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW("MESH", "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI("MESH", "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);
    }
    /* TODO handler for the failure */
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        memcpy(&mesh_parent_addr.addr, connected->connected.bssid, 6);
        ESP_LOGI("MESH",
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:"MACSTR"%s, ID:"MACSTR", duty:%d",
                 last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr), connected->duty);
        last_layer = mesh_layer;
        is_mesh_connected = true;
        if (esp_mesh_is_root()) {
            esp_netif_dhcpc_stop(netif_sta);
            esp_netif_dhcpc_start(netif_sta);
        }
        esp_mesh_comm_p2p_start();
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI("MESH",
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 disconnected->reason);
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
        
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        ESP_LOGI("MESH", "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
       
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        ESP_LOGI("MESH", "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
                 MAC2STR(root_addr->addr));
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        ESP_LOGI("MESH",
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        ESP_LOGI("MESH", "<MESH_EVENT_VOTE_STOPPED>");
        break;
    }
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        ESP_LOGI("MESH",
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        ESP_LOGI("MESH", "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(mesh_parent_addr.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        ESP_LOGI("MESH", "<MESH_EVENT_TODS_REACHABLE>state:%d", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        ESP_LOGI("MESH", "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        ESP_LOGI("MESH",
                 "<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        ESP_LOGI("MESH", "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI("MESH", "<MESH_EVENT_SCAN_DONE>number:%d",
                 scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        ESP_LOGI("MESH", "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
                 network_state->is_rootless);
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        ESP_LOGI("MESH", "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        ESP_LOGI("MESH", "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:"MACSTR"",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        ESP_LOGI("MESH", "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, "MACSTR"",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
    }
    break;
    case MESH_EVENT_PS_PARENT_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI("MESH", "<MESH_EVENT_PS_PARENT_DUTY>duty:%d", ps_duty->duty);
    }
    break;
    case MESH_EVENT_PS_CHILD_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI("MESH", "<MESH_EVENT_PS_CHILD_DUTY>cidx:%d, "MACSTR", duty:%d", ps_duty->child_connected.aid-1,
                MAC2STR(ps_duty->child_connected.mac), ps_duty->duty);
    }
    break;
    default:
        ESP_LOGI("MESH", "unknown id:%" PRId32 "", event_id);
        break;
    }
}


void setup() {
    #if (ENABLE_TCP_DISABLE_Serial2 == 1)


    #else
      size_t rxbufsize = Serial2.setRxBufferSize(4*1024); // Increased buffer size
      size_t txbufsize = Serial2.setTxBufferSize(2*1024); // Increased buffer size
      Serial2.begin(baudrate, SERIAL_8N1, 16, 17);


    #endif

    ESP_ERROR_CHECK(nvs_flash_init());
    /*  tcpip initialization */
    ESP_ERROR_CHECK(esp_netif_init());
    /*  event initialization */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /*  create network interfaces for mesh (only station instance saved for further manipulation, soft AP instance ignored */
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));
    /*  wifi initialization */
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
        /* Set the maximum Wi-Fi TX power */
    //ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));  // Set TX power to 20.5 dBm (maximum)
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));  // Set TX power to 20.5 dBm (maximum)
    /*  gets this esp mac address, to prevent sending message to itself */
    esp_wifi_get_mac(WIFI_IF_STA, my_address.addr);

    /*  mesh initialization */
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    /*  set mesh topology */
    ESP_ERROR_CHECK(esp_mesh_set_topology(MESH_TOPO_TREE));
    /*  set mesh max layer according to the topology */
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(6));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));

    /* Disable mesh PS function */
    ESP_ERROR_CHECK(esp_mesh_disable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));

    /* set this esp as root node*/ 
    ESP_ERROR_CHECK(esp_mesh_set_type(MESH_ROOT));
   
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    /* mesh ID */
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);
    /* router */
    cfg.channel = 0;
    // cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
    // memcpy((uint8_t *) &cfg.router.ssid, CONFIG_MESH_ROUTER_SSID, cfg.router.ssid_len);
    // memcpy((uint8_t *) &cfg.router.password, CONFIG_MESH_ROUTER_PASSWD,
    //        strlen(CONFIG_MESH_ROUTER_PASSWD));
    /* mesh softAP */
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(WIFI_AUTH_WPA_WPA2_PSK));
    cfg.mesh_ap.max_connection = 6;
    cfg.mesh_ap.nonmesh_max_connection = 0;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,
           strlen(CONFIG_MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

    /* mesh start */
    ESP_ERROR_CHECK(esp_mesh_start());
    ESP_LOGI("MESH", "mesh starts successfully, heap:%" PRId32 ", %s<%d>%s, ps:%d",  esp_get_minimum_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
             esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", esp_mesh_is_ps_enabled());
    
    #if (ENABLE_TCP_DISABLE_Serial2 == 1)
        server.begin();
        server.setNoDelay(true);
    #else 
        Serial2FlushRx();
    #endif
    


}

void loop()
{
    


}