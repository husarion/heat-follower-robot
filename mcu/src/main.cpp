#define F_CPU 168000000

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/image.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include "MLX90641_API.h"
#include "MLX90641_I2C_Driver.h"
#include "Motor.h"

#define NODE_NAME "Thermal_Camera"
#define LED_PIN PA2
#define MOTOR_PIN PA6
#define IMAGES 3 //number of camera shots per rotation

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
    }                              \
  }


rcl_publisher_t publisher, info_publisher;
rcl_subscription_t subscriber;
sensor_msgs__msg__Image msg;
std_msgs__msg__UInt16MultiArray info_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

static void rclc_spin_task(void *p);
static void publisher_task(void *p);
static void motorTask(void* p);
static void TriggerInterrupt(void);
static void uROS_Init();
static void uROS_DeInit();

enum infoDataType {RPM, TARGET, PID_I, PID_D, PID_E, FIN_PULSE_0, FIN_PULSE_1, FIN_PULSE_2, PULSE};
static paramsMLX90641 MLX_Params;
static uint8_t flag = 0;
static uint8_t imageId;
static uint16_t frameBuffer[16*12*IMAGES]; // [pixel0, pixel1, ...]
static uint16_t infoBuffer[10];
static uint16_t MLX_eeprom[832];
static uint16_t frameData[300];
static uint32_t lastPulseTickMS = 0;
static uint32_t currPulseTickMS = 0;
static float image[192];
static float MLX_Vdd = 0.0f;
static float MLX_Emmisivity = 0.0f;
static float MLX_Ambient = 0.0f;
volatile float targetRPM = 120;
volatile float rpm = 0;
volatile int pulses = 0;
int ret_code = 0;

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);

    if(rmw_uros_ping_agent(100, 1) == RCL_RET_OK)
    {
      uROS_DeInit();
      setup();
    }
  }
}

void uROS_Init()
{
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // create publishers
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image), "thermal_image"));
  RCCHECK(rclc_publisher_init_default(&info_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray), "thermal_camera_info"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

void uROS_DeInit()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rclc_support_fini(&support);
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
}

void setup() 
{
  portBASE_TYPE s1, s2, s3;
  
  MLX90641_I2CInit();
  Motor_Init();
  pinMode(LED_PIN, OUTPUT);
  pinMode(PA8, INPUT);
  pinMode(PA9, OUTPUT);
  pinMode(PA10, OUTPUT);
  attachInterrupt(PA8, TriggerInterrupt, FALLING);

  //image message definition 
  msg.header.frame_id.data = "map";
  msg.width = 16*IMAGES;
  msg.height = 12;
  msg.encoding.data = "mono16";
  msg.is_bigendian = false;
  msg.step = 32*IMAGES;
  msg.data.capacity = 12*16*IMAGES;
  msg.data.data = (uint8_t*)frameBuffer; //set frameBuffer as message data field
  msg.data.size = sizeof(frameBuffer);

  info_msg.data.capacity = 10;
  info_msg.data.data = (uint16_t*)infoBuffer;
  info_msg.data.size = 10;

  set_microros_transports(); //open serial communication 
  allocator = rcl_get_default_allocator();
  delay(1000); //wait for sensor to boot up
  
  //set image capture mode
  MLX90641_I2CWrite(0x33, 0x800D, 0x0901);
  delay(1000);
  
  //dump eeprom and check for errors
  ret_code = MLX90641_DumpEE(0x33, MLX_eeprom); 
  if(ret_code != 0)
  {
    while(1)
    {
      switch(ret_code)
      {
        case -1:
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
          delay(1000);      
        break;
        case -9:
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
          delay(500);      
        break;
        case -10:
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
          delay(50);      
        break;
        default:
        digitalWrite(LED_PIN, LOW);
        break;
      }
    }
  }

  digitalWrite(LED_PIN, HIGH); 
  MLX90641_SetResolution(0x33, 0);
  MLX90641_SynchFrame(0x33);

  int error = MLX90641_ExtractParameters(MLX_eeprom, &MLX_Params);
  if(error)
  {
    error_loop;
  }

  MLX90641_SetRefreshRate(0x33, 0x07);
  delay(10);
  MLX90641_SubpagesMode(0x33, 1);
  delay(10);
  
  //initialize variables
  MLX90641_GetFrameData(0x33, frameData);
  MLX_Vdd = MLX90641_GetVdd(frameData, &MLX_Params);
  MLX_Emmisivity = MLX90641_GetEmissivity(&MLX_Params);
  MLX_Ambient = MLX90641_GetTa(frameData, &MLX_Params);
  MLX_Params.alpha[36] = 10000;// broken pixel fix
  MLX90641_CalculateTo(frameData, &MLX_Params, MLX_Emmisivity, MLX_Ambient, image);

  uROS_Init();

  // create publisher task
  s1 = xTaskCreate(
      publisher_task, (const portCHAR *)"publisher_task",
      configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 2, NULL);

  // create spin task
  s2 = xTaskCreate(rclc_spin_task, "rclc_spin_task",
                   configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
                   NULL);

  //motor task
  s3 = xTaskCreate(motorTask, "motorTask", configMINIMAL_STACK_SIZE+1000, NULL, tskIDLE_PRIORITY+2, NULL);

  // check for creation errors
  if (s1 != pdPASS || s2 != pdPASS || s3!=pdPASS ) {
    while (1)
      ;
  }
  // start FreeRTOS
  vTaskStartScheduler();
}

static void rclc_spin_task(void *p) 
{
  UNUSED(p);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    rclc_executor_spin(&executor);
    vTaskDelayUntil(&xLastWakeTime, 1);
  }
}

static void publisher_task(void *p) 
{
  UNUSED(p);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  static uint16_t finishPulse[3];
  while (1) 
  { 
    //take a picture at specific position
    pulses = TIM2->CNT;
      taskENTER_CRITICAL();

    if( pulses == 20) 
    {
        imageId = 1;
    }
    else if(pulses == 110) 
    {
      
       imageId = 0;
    }
    else if(pulses == 245)
    {
        imageId = 2;
    }
    else 
    {
      imageId = 100;
    }
    taskEXIT_CRITICAL();

    // visualisation
    if(imageId<100)
    {
      // MLX90641_TriggerMeasurement(0x33);
      MLX90641_SynchFrame(0x33);
      taskENTER_CRITICAL();
      digitalWrite(PA9, HIGH);  
      vTaskDelay(10);
      MLX90641_GetFrameData(0x33, frameData);
      MLX_Ambient = MLX90641_GetTa(frameData, &MLX_Params);
      MLX90641_CalculateTo(frameData, &MLX_Params, MLX_Emmisivity, MLX_Ambient, image);
      finishPulse[imageId] = TIM2->CNT;
     // MLX90641_BadPixelsCorrection(36, image);
      MLX90641_BadPixelsCorrection(0, image);
      for(uint8_t i = 0; i< 192; i++)
      {
      //convert float to uint16 and mirror the image(from right to left)
      if(image[(int(i/16)*16 + 15) - (i - int(i/16)*16)] <0)
      {
        frameBuffer[int(i/16)*32 + i + (16*imageId)] = 0;
      }
      else
      {
        frameBuffer[int(i/16)*32 + i + (16*imageId)] = (uint16_t)(image[(int(i/16)*16 + 15) - (i - int(i/16)*16) ]*100); //magic
      }
      // frameBuffer[int(i/16)*32 + i + map(TIM2->CNT, 0, 290, 32, 0)] = (uint16_t)(image[(int(i/16)*16 + 15) - (i - int(i/16)*16) ]*100);
      // frameBuffer[i] = (uint16_t)(image[(int(i/16)*16 + 15) - (i - int(i/16)*16) ]*100);
      // frameBuffer[i] = (uint16_t)roundf(image[i]*100);
      }
      // for(uint8_t i = 0; i< 48; i++)
      // {
      //   frameBuffer[int(i/4)*44 + i + map(TIM2->CNT, 0, 290, 44, 0)] = (uint16_t)(image[(int(i/4)*16 + 9) - (i - int(i/4)*4) ]*100);
      // }
      taskEXIT_CRITICAL();
      if(imageId == 0)
      {
        rcl_publish(&publisher, &msg, NULL);
        rcl_publish(&info_publisher, &info_msg, NULL);
      } 
      infoBuffer[FIN_PULSE_0] = finishPulse[0];
      infoBuffer[FIN_PULSE_1] = finishPulse[1];
      infoBuffer[FIN_PULSE_2] = finishPulse[2];
    }
    // vTaskDelayUntil(&xLastWakeTime, 1);
  }
}
static void motorTask(void* p)
{
  UNUSED(p);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float error = 0.0f;
  float lastError = 1.0f;
  float derivative = 1.0f;
  float integral = 1.0f;
  float kp=0.8f;
  float ki=0.65f;
  float kd=4.0f;
  uint16_t pwm = 0;
  while(1)
  {    
    uint32_t interPulseTime = currPulseTickMS - lastPulseTickMS; // rotation time in milliseconds
    rpm = (float)(1000.0f/interPulseTime)*60.0f;
    if((HAL_GetTick() - currPulseTickMS) > 1500)
    {
      rpm = 0;
    }
    //pid
    error = targetRPM - rpm;
    integral = integral + error;
    derivative = error - lastError;
    pwm = (kp*error) + (ki*integral) + (kd*derivative);

    if(pwm>5000)pwm = 5000;
    if(pwm<0)pwm = 0;

    Motor_Run(pwm);
    lastError = error;

    infoBuffer[RPM] = rpm;
    infoBuffer[TARGET] = targetRPM;
    infoBuffer[PID_E] = error;
    infoBuffer[PID_I] = integral;
    infoBuffer[PID_D] = derivative;
    vTaskDelayUntil(&xLastWakeTime, 175);
  }
}
void loop() {

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(1000);
}

static void TriggerInterrupt()
{
  //rotation complete interrupt
  infoBuffer[PULSE] = TIM2->CNT;
  TIM2->CNT = 0;
  
  lastPulseTickMS = currPulseTickMS;
  currPulseTickMS = HAL_GetTick();
}

void TimerInterrupt()
{

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}