# Setup Instructions for Micro-ROS on ESP32-S3 and ROS2 for the Mechanical Manipulator of Lakbay Rover

This repository contains the firmware and setup instructions for running **Micro-ROS** on an **ESP32-S3** to control the **6-DOF mechanical manipulator**.  
It enables the ESP32-S3 to communicate with a ROS 2 system using a **Micro-ROS Agent**, providing **real-time control, telemetry, and sensor integration**.

> ⚠️ **Note**: ESP32 (non-S3) *may* work, but this setup is **tested and verified only on ESP32-S3**.

---

## Table of Contents

1. [Requirements](#requirements)  
2. [Project Overview](#project-overview)  
3. [Setup Instructions](#setup-instructions)  
4. [Usage](#usage)  
5. [Troubleshooting](#troubleshooting)  

---



## Requirements

- **ESP32-S3 development board**
- **Ubuntu 24.04.x LTS** ([Installation Guide](https://ubuntu.com/download/desktop))  
- **ESP-IDF v5.x VS code extension** 
- **ROS 2 Jazzy Jalisco** ([Installation Guide](https://docs.ros.org/en/jazzy/Installation.html))  
- **Micro-ROS Agent** ([Installation Guide](https://micro.ros.org/docs/tutorials/core/first_application_linux/))  

---

## Project Overview

The ESP32-S3 firmware connects to a ROS 2 network using **Micro-ROS**. The workflow is:

ESP32-S3 (Micro-ROS Client) <br>
│ <br>
▼ <br>
Micro-ROS Agent (PC) <br>
│ <br>
▼ <br>
ROS 2 Network <br>
│ <br>
▼ <br>
RViz / Control Nodes <br>


Key features:

- Communication with ROS 2 topics and services  
- Sensor feedback and actuator control  
- Ethernet or Wi-Fi network support via custom network interface  

---

## Setup Instructions

### Clone the Repository

Open a terminal and run:

```bash
git clone https://github.com/junjun-25/lakbay-mechanical-manipulator.git

cd /lakbay-mechanical-manipulator/Mechanical manipulator/components
```



### Clone the Micro-ROS ESP-IDF Component
Inside the components folder:
```bash
git clone https://github.com/micro-ROS/micro_ros_espidf_component.git

cd micro_ros_espidf_component
```

### Clean the Micro-ROS Library

```bash
make -f libmicroros.mk clean
```

### Replace the Network Interface Code
Navigate to the network interface directory:
```bash
cd network_interfaces
```

Replace the contents of uros_ethernet_netif.c with the following code:

```c

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "lwip/ip_addr.h"
#if CONFIG_ETH_USE_SPI_ETHERNET
#include "driver/spi_master.h"
#endif
#include "uros_network_interfaces.h"

#ifdef CONFIG_MICRO_ROS_ESP_NETIF_ENET

uint8_t IP_ADDRESS[4];

static const char *TAG = "eth_interface";

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");

    IP_ADDRESS[0] = esp_ip4_addr_get_byte(&event->ip_info.ip, 0);
    IP_ADDRESS[1] = esp_ip4_addr_get_byte(&event->ip_info.ip, 1);
    IP_ADDRESS[2] = esp_ip4_addr_get_byte(&event->ip_info.ip, 2);
    IP_ADDRESS[3] = esp_ip4_addr_get_byte(&event->ip_info.ip, 3);
}

esp_err_t uros_network_interface_initialize(void)
{
    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_MICRO_ROS_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_MICRO_ROS_ETH_PHY_RST_GPIO;
#if CONFIG_MICRO_ROS_USE_INTERNAL_ETHERNET
    // Init MAC and PHY configs to default
    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    mac_config.sw_reset_timeout_ms = 250; // Needed for some ethernet controllers to initiate

    emac_config.smi_mdc_gpio_num = CONFIG_MICRO_ROS_ETH_MDC_GPIO;
    emac_config.smi_mdio_gpio_num = CONFIG_MICRO_ROS_ETH_MDIO_GPIO;
#if CONFIG_MICRO_ROS_ETH_PHY_IP101
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);
#elif CONFIG_MICRO_ROS_ETH_PHY_RTL8201
    esp_eth_phy_t *phy = esp_eth_phy_new_rtl8201(&phy_config);
#elif CONFIG_MICRO_ROS_ETH_PHY_LAN8720
    esp_eth_phy_t *phy = esp_eth_phy_new_lan8720(&phy_config);
#elif CONFIG_MICRO_ROS_ETH_PHY_DP83848
    esp_eth_phy_t *phy = esp_eth_phy_new_dp83848(&phy_config);
#elif CONFIG_MICRO_ROS_ETH_PHY_KSZ8041
    esp_eth_phy_t *phy = esp_eth_phy_new_ksz8041(&phy_config);
#endif
#elif CONFIG_ETH_USE_SPI_ETHERNET
    gpio_install_isr_service(0);
    spi_device_handle_t spi_handle = NULL;
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_MICRO_ROS_ETH_SPI_MISO_GPIO,
        .mosi_io_num = CONFIG_MICRO_ROS_ETH_SPI_MOSI_GPIO,
        .sclk_io_num = CONFIG_MICRO_ROS_ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_MICRO_ROS_ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
#if CONFIG_MICRO_ROS_USE_DM9051
    spi_device_interface_config_t devcfg = {
        .command_bits = 1,
        .address_bits = 7,
        .mode = 0,
        .clock_speed_hz = CONFIG_MICRO_ROS_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = CONFIG_MICRO_ROS_ETH_SPI_CS_GPIO,
        .queue_size = 20
    };
    ESP_ERROR_CHECK(spi_bus_add_device(CONFIG_MICRO_ROS_ETH_SPI_HOST, &devcfg, &spi_handle));
    /* dm9051 ethernet driver is based on spi driver */
    eth_dm9051_config_t dm9051_config = ETH_DM9051_DEFAULT_CONFIG(CONFIG_MICRO_ROS_ETH_SPI_HOST, &devcfg);
    dm9051_config.int_gpio_num = CONFIG_MICRO_ROS_ETH_SPI_INT_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_dm9051(&dm9051_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_dm9051(&phy_config);
#elif CONFIG_MICRO_ROS_USE_W5500
    spi_device_interface_config_t devcfg = {
        .command_bits = 16, // Actually it's the address phase in W5500 SPI frame
        .address_bits = 8,  // Actually it's the control phase in W5500 SPI frame
        .mode = 0,
        .clock_speed_hz = CONFIG_MICRO_ROS_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = CONFIG_MICRO_ROS_ETH_SPI_CS_GPIO,
        .queue_size = 20
    };
    ESP_ERROR_CHECK(spi_bus_add_device(CONFIG_MICRO_ROS_ETH_SPI_HOST, &devcfg, &spi_handle));
    /* w5500 ethernet driver is based on spi driver */
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(CONFIG_MICRO_ROS_ETH_SPI_HOST, &devcfg);
    w5500_config.int_gpio_num = CONFIG_MICRO_ROS_ETH_SPI_INT_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
#endif
#endif // CONFIG_ETH_USE_SPI_ETHERNET
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));
#if CONFIG_ETH_USE_SPI_ETHERNET
    /* The SPI Ethernet module might doesn't have a burned factory MAC address, we cat to set it manually.
       02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.
    */
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
        0x02, 0x00, 0x00, 0x12, 0x34, 0x56
    }));
#endif
    /* attach Ethernet driver to TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    // ---------- STATIC IP CONFIGURATION ----------
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip,      192, 168, 1, 3);   // Device IP (ESP32)
    IP4_ADDR(&ip_info.gw,      192, 168, 1, 10);   // Gateway (PC running micro-ROS agent)
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0); // Subnet mask
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netif)); // Stop DHCP client
    ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif, &ip_info));
    /* start Ethernet driver state machine */
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    return ESP_OK;
}

#endif
```



⚠️ Important

   * Ensure the IP addresses match your network.


###Return to the Main Project Directory

```bash
cd ../..
```

### Open the Project in VS Code

```bash
code .
```
Open the ESP-IDF Terminal inside VS Code. <br>

### Edit CMakeLists.txt

Navigate through the following file then edit it:

micro_ros_espidf_component/CMakeLists.txt

```cmake

idf_component_register(SRCS ${COMPONENT_SRC}
                       INCLUDE_DIRS "network_interfaces"
                       REQUIRES nvs_flash esp_wifi esp_eth esp_netif lwip driver)
```

### Build the Project
Run the following commands in order:

```bash
idf.py add-dependency "espressif/esp-dsp"
idf.py fullclean
idf.py reconfigure
idf.py build

```

### Create a ROS 2 Workspace

```bash
mkdir -p ~/robot_controller/sr

cd ~/robot_controller/src
```

### Add the robot_description Package

Move the robot_description folder into the src directory:
Your workspace structure should look like this:

```text
robot_controller/
└── src/
    └── robot_description/
        ├── urdf/
        ├── launch/
        ├── meshes/
        ├── config/
        └── package.xml
```

### Build the Workspace
```bash
cd ~/robot_controller

colcon build --symlink-install
```


### Source the Workspace
After a successful build:
```bash
source ~/robot_controller/robot_description/install/setup.bash
```

ros2 launch robot_description display.launch.py

## Usage

1. Flash the firmware to the ESP32-S3 using ESP-IDF.

2. Start the Micro-ROS Agent on your PC:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

3. Power the ESP32-S3 and verify connection.
4. Plug in a USB Joystick on your pc.
5. Launch the Robot Controller and Model in RViz.

```bash
ros2 launch robot_description display.launch.py
```
This will:

* Load the robot URDF
* Open RViz with the manipulator model displayed
* Load the controllers

## Troubleshooting

* ESP32 not connecting

  * Verify static IP configuration

  * Check Ethernet wiring or SPI Ethernet module

  * Ensure the Micro-ROS agent is running first

* Build errors

  * Confirm ESP-IDF version is v5.x

  * Run idf.py fullclean before rebuilding
 


https://github.com/user-attachments/assets/c62c71a4-ae30-43b2-b577-fd333899c85b


