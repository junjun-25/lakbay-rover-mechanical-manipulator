#include <stdio.h>
#include "matrix.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_mac.h"
#include <driver/uart.h>
#include "driver/gptimer.h"
#include "freertos/semphr.h"
#include <esp_dsp.h>
#include "esp_twai.h"

/* micro-ROS headers */
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "sensor_msgs/msg/joy.h"
#include "sensor_msgs/msg/joint_state.h"
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>


/* Custom headers */
#include "inverse_kinematics.h"
#include "inverse_dynamics.h"


#define TIMER1_INTERVAL_US (1000) // 1khz
#define TIMER2_INTERVAL_US (2000) // 500hz

static const char *TAG = "micro_ros";

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    #include <rmw_microros/rmw_microros.h>
#endif
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    #include <rmw_microxrcedds_c/config.h>
    #include "esp32_serial_transport.h"
    static uart_port_t uart_port = UART_NUM_1;
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)) { \
        printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
        esp_restart(); \
        vTaskDelete(NULL); \
    } \
}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)) { \
        printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        return; \
    } \
}

/* Subscription and message */
rcl_subscription_t joy_subscription;
rcl_publisher_t   joint_state_publisher;

sensor_msgs__msg__Joy joy_msg;
sensor_msgs__msg__JointState joint_state_msg;
sensor_msgs__msg__Joy joy_copy;


//---- TASK HANDLER----//
TaskHandle_t Controller_task = NULL;
TaskHandle_t Micro_Ros_task = NULL;
TaskHandle_t Can_Bus_task = NULL;

//---- SEMAPHORES & MUTEX ----//
static SemaphoreHandle_t executor_semaphore = NULL;
static SemaphoreHandle_t controller_semaphore = NULL;
static SemaphoreHandle_t joy_mutex;

static SemaphoreHandle_t q_mutex;

static float q_shared[6] = {0};


volatile uint32_t msg_count = 0;
int64_t last_time_us = 0;

volatile bool joy_received = false;
volatile bool mode_select = false;


bool ping_agent()
{
    return (rmw_uros_ping_agent(1000, 1) == RMW_RET_OK);
}

void reconnect_to_agent()
{
    ESP_LOGW(TAG, "Attempting to reconnect to micro-ROS Agent...");
    while (!ping_agent()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "Agent reachable. Resuming micro-ROS task.");
    esp_restart(); 
}

void rotm_to_rpy(float *R, float *roll, float *pitch, float *yaw)
{
    *pitch = atan2f(-R[6], sqrtf((R[0]*R[0])+ (R[3]* R[3])));

    if (fabsf(R[0]) < 1e-6f && fabsf(R[3]) < 1e-6f) {

        *yaw = 0.0f;
        *roll = atan2f(R[1], R[4]);

    } else {

        *roll = atan2f(R[7], R[8]);
        *yaw  = atan2f(R[3], R[0]);
    }
}


void joy_callback(const void * msgin) {
    const sensor_msgs__msg__Joy * msg = (const sensor_msgs__msg__Joy *)msgin;

    if (xSemaphoreTake(joy_mutex, portMAX_DELAY) == pdTRUE)  {
        for (size_t i = 0; i < msg->axes.size; i++) {
            if (i < joy_copy.axes.size)
                joy_copy.axes.data[i] = msg->axes.data[i];
        }
        for (size_t i = 0; i < msg->buttons.size; i++) {
            if (i < joy_copy.buttons.size)
                joy_copy.buttons.data[i] = msg->buttons.data[i];
        }

        xSemaphoreGive(joy_mutex);
        joy_received = true;
    }
}

static bool IRAM_ATTR timer1_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (executor_semaphore != NULL) {
        xSemaphoreGiveFromISR(executor_semaphore, &xHigherPriorityTaskWoken);
    }
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
    return true;
}

static bool IRAM_ATTR timer2_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (controller_semaphore != NULL) {
        xSemaphoreGiveFromISR(controller_semaphore, &xHigherPriorityTaskWoken);
    }
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
    return true;
}

void hardware_timer_setup(uint64_t time_in_us, bool (*callback)(gptimer_handle_t, const gptimer_alarm_event_data_t *, void *))
{
    gptimer_handle_t gptimer = NULL;

    // Create the timer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, 
        .direction = GPTIMER_COUNT_UP,     
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = time_in_us, 
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

void init_joint_state() {
    size_t joint_count = 6;

    sensor_msgs__msg__JointState__init(&joint_state_msg);

    //rosidl_runtime_c__String__init(&joint_state_msg.header.frame_id);
    //rosidl_runtime_c__String__assign(&joint_state_msg.header.frame_id, "base_link");

    //joint_state_msg.header.stamp.sec = 0;
    //joint_state_msg.header.stamp.nanosec = 0;

    rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, joint_count);
    for (size_t i = 0; i < joint_count; i++) {
        rosidl_runtime_c__String__init(&joint_state_msg.name.data[i]);
        char joint_name[16];
        snprintf(joint_name, sizeof(joint_name), "joint%zu", i + 1);
        rosidl_runtime_c__String__assign(&joint_state_msg.name.data[i], joint_name);
    }

    rosidl_runtime_c__double__Sequence__init(&joint_state_msg.position, joint_count);
    rosidl_runtime_c__double__Sequence__init(&joint_state_msg.velocity, 0);
    rosidl_runtime_c__double__Sequence__init(&joint_state_msg.effort, 0);

    joint_state_msg.position.size = joint_count;
    //joint_state_msg.velocity.size = joint_count;
    //joint_state_msg.effort.size   = joint_count;

}


void publish_joint_states_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
	if (timer != NULL){

        if (xSemaphoreTake(q_mutex, 0) == pdTRUE) {

            for (int i = 0; i < 6; i++) {
                joint_state_msg.position.data[i] = q_shared[i];
            }

            xSemaphoreGive(q_mutex);
            //int64_t now_us = esp_timer_get_time(); // microseconds since boot
            
            //joint_state_msg.header.stamp.sec = (int32_t)(now_us / 1000000ULL);
            //joint_state_msg.header.stamp.nanosec = (uint32_t)((now_us % 1000000ULL) * 1000ULL);

            RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
        }
    }
}


void micro_ros_task(void * arg) 
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

        // Static Agent IP and port can be used instead of autodisvery.
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
        //RCCHECK(rmw_uros_discover_agent(rmw_options));
    #endif

    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    /* Create a micro-ROS node */
    rcl_node_t node;
    
    RCCHECK(rclc_node_init_default(&node, "robot_controller", "", &support));

    sensor_msgs__msg__Joy__init(&joy_msg);
    rosidl_runtime_c__float__Sequence__init(&joy_msg.axes, 6);
    rosidl_runtime_c__int32__Sequence__init(&joy_msg.buttons, 12);

    sensor_msgs__msg__Joy__init(&joy_copy);
    rosidl_runtime_c__float__Sequence__init(&joy_copy.axes, 6);
    rosidl_runtime_c__int32__Sequence__init(&joy_copy.buttons, 12);

    init_joint_state();

    RCCHECK(rclc_publisher_init_best_effort(
        &joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states"));
    
    RCCHECK(rclc_subscription_init_default(
        &joy_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
        "joy")); 

	rcl_timer_t timer;
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default2(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		publish_joint_states_callback,
		true));

    /* Executor */
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &joy_subscription, &joy_msg, &joy_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while (1) {
        if (xSemaphoreTake(executor_semaphore, portMAX_DELAY) == pdTRUE) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
        }

        static int64_t last_ping_time = 0;
        int64_t now_us = esp_timer_get_time();
        if (last_ping_time == 0) last_ping_time = now_us;

        if (now_us - last_ping_time >= 500000) { 
            if (!ping_agent()) {
                reconnect_to_agent();
            }
            last_ping_time = now_us;
        }
    }
}

void can_task(void *arg)
{
    while(1){

    vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void controller_task(void *arg)
{

    sensor_msgs__msg__Joy local_joy;
    sensor_msgs__msg__Joy__init(&local_joy);
    rosidl_runtime_c__float__Sequence__init(&local_joy.axes, 6);
    rosidl_runtime_c__int32__Sequence__init(&local_joy.buttons, 12);
    float q_current[6] = {0};
    float q[6] = {0};
        
    float T[16];

    forward_kinematics(q_current, T);

    float R_fk[9];

    float rotmat[9];
    rotmat[0] = T[0]; rotmat[1] = T[1]; rotmat[2] = T[2];
    rotmat[3] = T[4]; rotmat[4] = T[5]; rotmat[5] = T[6];
    rotmat[6] = T[8]; rotmat[7] = T[9]; rotmat[8] = T[10];

    static const float r_offset[9] = {0,0,1, 0,-1,0, 1,0,0};

    dspm_mult_3x3x3_f32(rotmat, r_offset, R_fk);

    float input_x = T[3];
    float input_y = T[7];
    float input_z = T[11];
    float input_roll;
    float input_pitch;
    float input_yaw;

    rotm_to_rpy(R_fk, &input_roll, &input_pitch, &input_yaw); 


    float prev_input_x = input_x;
    float prev_input_y = input_y;
    float prev_input_z = input_z;
    float prev_input_roll = input_roll;
    float prev_input_pitch = input_pitch;
    float prev_input_yaw = input_yaw;

    float prev_linear_vx = 0.0f;
    float prev_linear_vy = 0.0f;
    float prev_linear_vz = 0.0f;
    float prev_angular_vx = 0.0f;
    float prev_angular_vy = 0.0f;
    float prev_angular_vz = 0.0f;


    int prev_mode_select_button = 0;
    
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;

    float r_max = 1300.0f;

    //-----------HOME_POSE------------//
    float q_home [6] = {0.0f, 1.5f, -2.0f, 0.0f, 1.5f, 0.0f};

    float qdot[6] = {0};  // joint velocities
    float qddot[6]= {0};  // joint accelerations

    float tau[6];

    const casadi_real* casadi_arg[3] = {q, qdot, qddot};
    casadi_real* res[1] = {tau};


    int mem = inverse_dynamics_rnea_alloc_mem();
    inverse_dynamics_rnea_init_mem(mem);

    while(1){
        if (xSemaphoreTake(controller_semaphore, portMAX_DELAY) == pdTRUE) {
            
            if (!joy_received) {
                vTaskDelay(pdMS_TO_TICKS(5)); 
                continue;
            }

            if (xSemaphoreTake(joy_mutex, portMAX_DELAY) == pdTRUE) {
                for (size_t i = 0; i < local_joy.axes.size; i++)
                    local_joy.axes.data[i] = joy_copy.axes.data[i];

                for (size_t i = 0; i < local_joy.buttons.size; i++)
                    local_joy.buttons.data[i] = joy_copy.buttons.data[i];

                xSemaphoreGive(joy_mutex);
            }


            static uint64_t prev_time_us = 0; 
            uint64_t current_time_us = esp_timer_get_time(); 
            double dt = 0.0;

            if (prev_time_us > 0) {
                dt = (current_time_us - prev_time_us) / 1000000.0;
            }


            int button_select = local_joy.buttons.data[8];
            if (button_select == 1 && prev_mode_select_button == 0) {
                mode_select = !mode_select;
                printf("MODE SELECTED: %s\n", mode_select ? "JOINT JOG" : "TWIST");
                if (!mode_select) {
                    forward_kinematics(q_current, T);

                    float R_fk[9];

                    float rotmat[9];
                    rotmat[0] = T[0]; rotmat[1] = T[1]; rotmat[2] = T[2];
                    rotmat[3] = T[4]; rotmat[4] = T[5]; rotmat[5] = T[6];
                    rotmat[6] = T[8]; rotmat[7] = T[9]; rotmat[8] = T[10];

                    static const float r_offset[9] = {0,0,1, 0,-1,0, 1,0,0};

                    dspm_mult_3x3x3_f32(rotmat, r_offset, R_fk);

                    input_x = T[3];
                    input_y = T[7];
                    input_z = T[11];

                    prev_input_x = input_x;
                    prev_input_y = input_y;
                    prev_input_z = input_z;

                    prev_input_roll = input_roll;
                    prev_input_pitch = input_pitch;
                    prev_input_yaw = input_yaw;

                } else {
                    IK(input_x, input_y, input_z,
                       qx, qy, qz, qw,
                       q_current, q);
                }
            }
            prev_mode_select_button = button_select;


            if (!mode_select) {

                float linear_velocity;
                float angular_velocity;

                if (local_joy.buttons.data[4] == 0 ) {
                    linear_velocity = 100.0f;
                    angular_velocity = 0.5f;
                } else {
                    linear_velocity = 400.0f;
                    angular_velocity = 2.0f;
                }

                float linear_vx = linear_velocity * local_joy.axes.data[1];
                float linear_vy = linear_velocity * local_joy.axes.data[0];
                float linear_vz = linear_velocity * local_joy.axes.data[3];
                float angular_vx = angular_velocity * local_joy.axes.data[2];
                float angular_vy = angular_velocity * -(local_joy.axes.data[5]);
                float angular_vz = angular_velocity * local_joy.axes.data[4];

                float deadzone = 0.05f;
                if (fabs(linear_vx)  < deadzone) linear_vx  = 0.0f;
                if (fabs(linear_vy)  < deadzone) linear_vy  = 0.0f;
                if (fabs(linear_vz)  < deadzone) linear_vz  = 0.0f;

                if (fabs(angular_vx) < deadzone) angular_vx = 0.0f;
                if (fabs(angular_vy) < deadzone) angular_vy = 0.0f;
                if (fabs(angular_vz) < deadzone) angular_vz = 0.0f;

                input_x += 0.5f * (linear_vx + prev_linear_vx) * dt;
                input_y += 0.5f * (linear_vy + prev_linear_vy) * dt;
                input_z += 0.5f * (linear_vz + prev_linear_vz) * dt;

                input_roll  += 0.5f * (angular_vx + prev_angular_vx) * dt;
                input_pitch += 0.5f * (angular_vy + prev_angular_vy) * dt;
                input_yaw   += 0.5f * (angular_vz + prev_angular_vz) * dt;

                input_x = clampf(input_x, -1300.0f, 1300.0f);
                input_y = clampf(input_y, -1300.0f, 1300.0f);
                input_z = clampf(input_z, -1000.0f, 1300.0f);
                input_roll = clampf(input_roll, -3.14f, 3.14f);
                input_pitch = clampf(input_pitch, -2.0f, 2.0f);
                input_yaw = clampf(input_yaw, -3.14f, 3.14f);

                float r = sqrtf((input_x*input_x) + (input_y*input_y) + (input_z*input_z) );

                if (r > 1e-12) {
                    if (r > r_max) {
                        float scale = r_max / r;
                        input_x *= scale;
                        input_y *= scale;
                        input_z *= scale;
                    } 
                } else {
                    input_x = prev_input_x;
                    input_y = prev_input_y;
                    input_z = prev_input_z;
                }

                // save previous velocities
                prev_linear_vx  = linear_vx;
                prev_linear_vy  = linear_vy;
                prev_linear_vz  = linear_vz;

                prev_angular_vx = angular_vx;
                prev_angular_vy = angular_vy;
                prev_angular_vz = angular_vz;

                //ESP_LOGI("controller_task",
                //       "Position: x=%.3f y=%.3f z=%.3f | Orientation: roll=%.3f pitch=%.3f yaw=%.3f | dt=%.3f",
                //    input_x, input_y, input_z,
                //    input_roll, input_pitch, input_yaw, dt);
    
                qx = sinf(input_roll/2) * cosf(input_pitch/2) * cosf(input_yaw/2) - cosf(input_roll/2) * sinf(input_pitch/2) * sinf(input_yaw/2);
                qy = cosf(input_roll/2) * sinf(input_pitch/2) * cosf(input_yaw/2) + sinf(input_roll/2) * cosf(input_pitch/2) * sinf(input_yaw/2);
                qz = cosf(input_roll/2) * cosf(input_pitch/2) * sinf(input_yaw/2) - sinf(input_roll/2) * sinf(input_pitch/2) * cosf(input_yaw/2);
                qw = cosf(input_roll/2) * cosf(input_pitch/2) * cosf(input_yaw/2) + sinf(input_roll/2) * sinf(input_pitch/2) * sinf(input_yaw/2);

                IK(input_x, input_y, input_z, qx, qy, qz, qw, q_current, q);

                
                if (within_limits){
                    prev_input_x = input_x;
                    prev_input_y = input_y;
                    prev_input_z = input_z;
                    prev_input_roll = input_roll;
                    prev_input_pitch = input_pitch;
                    prev_input_yaw = input_yaw;

                } else {
                    input_x = prev_input_x;
                    input_y = prev_input_y;
                    input_z = prev_input_z;
                    input_roll = prev_input_roll;
                    input_pitch = prev_input_pitch;
                    input_yaw = prev_input_yaw;
                }


            } else {

                float joint_velocity;

                if (local_joy.buttons.data[4] == 0 ) {
                    joint_velocity = 1;
                } else {
                    joint_velocity = 4;
                }

                float step = 0.2; 
                q[0] += step * local_joy.axes.data[0] * joint_velocity * dt;
                q[1] += step * local_joy.axes.data[1] * joint_velocity* dt;
                q[2] += step * local_joy.axes.data[3] * joint_velocity* dt;
                q[3] += step * -local_joy.axes.data[2] * joint_velocity * dt;
                q[4] += step * local_joy.axes.data[5] * joint_velocity * dt;
                q[5] += step * -local_joy.axes.data[4] * joint_velocity * dt;

                forward_kinematics(q, T);

                rotmat[0] = T[0]; rotmat[1] = T[1]; rotmat[2] = T[2];
                rotmat[3] = T[4]; rotmat[4] = T[5]; rotmat[5] = T[6];
                rotmat[6] = T[8]; rotmat[7] = T[9]; rotmat[8] = T[10];

                dspm_mult_3x3x3_f32(rotmat, r_offset, R_fk);

                input_x = T[3];
                input_y = T[7];
                input_z = T[11];
                rotm_to_rpy(R_fk, &input_roll, &input_pitch, &input_yaw); 
                //printf("orientation : x:%f, y:%f, z:%f\n", input_roll, input_pitch, input_yaw);
            }

            if (xSemaphoreTake(q_mutex, portMAX_DELAY) == pdTRUE) {
                for (int i = 0; i < 6; i++) {
                    q_shared[i] = q[i];   // radians
                }
                xSemaphoreGive(q_mutex);
            }
            
            for (int i=0; i<6; i++) {
                qdot[i] = (q[i] - q_current[i]) / dt;
            }

            memcpy(q_current, q, sizeof(q));

            //ESP_LOGI("controller_task","position: q1:%f q2:%f q3:%f q4:%f q5:%f q6:%f", q_current[0], q_current[1], q_current[2], q_current[3], q_current[4],q_current[5]);
            prev_time_us = current_time_us;
            static int loop_counter = 0;
            inverse_dynamics_rnea(casadi_arg, res, NULL, NULL, mem);

            loop_counter++;
            if (loop_counter >= 100) {
                loop_counter = 0;  // reset counter
                printf("Joint Torques: ");
                for (int i = 0; i < 6; ++i) {
                    printf("%f ", tau[i]);
                }
                printf("\n");
            }
        }
    }
    inverse_dynamics_rnea_free_mem(mem);
}

void app_main(void)
{
    #if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(
        true,
        (void *) &uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read
    );
    #elif defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #else
        #error micro-ROS transports misconfigured
    #endif  
    
    executor_semaphore = xSemaphoreCreateBinary();
    controller_semaphore = xSemaphoreCreateBinary();
    joy_mutex = xSemaphoreCreateMutex();
    q_mutex = xSemaphoreCreateMutex();

    if (executor_semaphore == NULL || controller_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphores\n");
        return;
    }
    if ( q_mutex == NULL || joy_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex\n");
        return;
    }

    xTaskCreatePinnedToCore(micro_ros_task, "uros_task", 16000, NULL, 10, &Micro_Ros_task, 0);
    xTaskCreatePinnedToCore(controller_task, "controller_task", 8192, NULL, 9, &Controller_task, 1);
    xTaskCreatePinnedToCore(can_task, "can_task", 4096, NULL, 8, &Can_Bus_task, 0);

    hardware_timer_setup(TIMER1_INTERVAL_US, timer1_callback);
    hardware_timer_setup(TIMER2_INTERVAL_US, timer2_callback);

    vTaskDelete(NULL);
    
}
