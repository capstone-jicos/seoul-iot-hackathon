// ----------------------------------------------------------------------------
// Copyright 2016-2018 ARM Ltd.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------
#include "XNucleoIKS01A2.h"
#include "simplem2mclient.h"
#ifdef TARGET_LIKE_MBED
#include "mbed.h"
#endif
#include "application_init.h"
#include "common_button_and_led.h"
#include "blinky.h"

DigitalOut led(LED2);
AnalogIn CDS(A0);
AnalogIn Pressure(A3);

static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);
// event based LED blinker, controlled via pattern_resource
static Blinky blinky;
static HTS221Sensor *hum_temp = mems_expansion_board->ht_sensor;


static void main_application(void);

int main(void)
{
    mcc_platform_run_program(main_application);
}

// Pointers to the resources that will be created in main_application().
static M2MResource* button_res;
static M2MResource* pattern_res;
static M2MResource* temp_res;
static M2MResource* hum_res;
static M2MResource* cds_res;
static M2MResource* led_res;

// Pointer to mbedClient, used for calling close function.
static SimpleM2MClient *client;

void pattern_updated(const char *)
{
    printf("PUT received, new value: %s\n", pattern_res->get_value_string().c_str());
}

void blink_callback(void *)
{
    String pattern_string = pattern_res->get_value_string();
    const char *pattern = pattern_string.c_str();
    printf("LED pattern = %s\n", pattern);

    // The pattern is something like 500:200:500, so parse that.
    // LED blinking is done while parsing.
    const bool restart_pattern = false;
    if (blinky.start((char*)pattern_res->value(), pattern_res->value_length(), restart_pattern) == false) {
        printf("out of memory error\n");
    }
}
/* Helper function for printing floats & doubles */
static char *print_double(char* str, double v, int decimalDigits=2)
{
	  int i = 1;
	    int intPart, fractPart;
		  int len;
		    char *ptr;
			 
			   /* prepare decimal digits multiplicator */
			     for (;decimalDigits!=0; i*=10, decimalDigits--);
				  
				    /* calculate integer & fractinal parts */
					  intPart = (int)v;
					    fractPart = (int)((v-(double)(int)v)*i);
						 
						   /* fill in integer part */
						     sprintf(str, "%i.", intPart);
							  
							    /* prepare fill in of fractional part */
								  len = strlen(str);
								    ptr = &str[len];
									 
									   /* fill in leading fractional zeros */
									     for (i/=10;i>1; i/=10, ptr++) {
											     if (fractPart >= i) {
													       break;
														       }
															       *ptr = '0';
																     }
																	  
																	    /* fill in (rest of) fractional part */
																		  sprintf(ptr, "%i", fractPart);
																		   
																		     return str;
}
void button_notification_status_callback(const M2MBase& object, const NoticationDeliveryStatus status)
{
    switch(status) {
        case NOTIFICATION_STATUS_BUILD_ERROR:
            printf("Notification callback: (%s) error when building CoAP message\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_RESEND_QUEUE_FULL:
            printf("Notification callback: (%s) CoAP resend queue full\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_SENT:
            printf("Notification callback: (%s) Notification sent to server\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_DELIVERED:
            printf("Notification callback: (%s) Notification delivered\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_SEND_FAILED:
            printf("Notification callback: (%s) Notification sending failed\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_SUBSCRIBED:
            printf("Notification callback: (%s) subscribed\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_UNSUBSCRIBED:
            printf("Notification callback: (%s) subscription removed\n", object.uri_path());
            break;
        default:
            break;
    }
}

// This function is called when a POST request is received for resource 5000/0/1.
void unregister(void *)
{
    printf("Unregister resource executed\n");
    client->close();
}

// This function is called when a POST request is received for resource 5000/0/2.
void factory_reset(void *)
{
    printf("Factory reset resource executed\n");
    client->close();
    kcm_status_e kcm_status = kcm_factory_reset();
    if (kcm_status != KCM_STATUS_SUCCESS) {
        printf("Failed to do factory reset - %d\n", kcm_status);
    } else {
        printf("Factory reset completed. Now restart the device\n");
    }
}

void main_application(void)
{
    // https://github.com/ARMmbed/sd-driver/issues/93 (IOTMORF-2327)
    // SD-driver initialization can fails with bd->init() -5005. This wait will
    // allow the board more time to initialize.
 float value1=0, old_val1=25;
 float value2=0, old_val2=30;
 float cdsVal=0;
 float sensorReading=0;
 char buffer1[32],buffer2[32],buffer3[32], buffer4[32];

#ifdef TARGET_LIKE_MBED
    wait(2);
#endif
    // Initialize trace-library first
    if (application_init_mbed_trace() != 0) {
        printf("Failed initializing mbed trace\n" );
        return;
    }

    // Initialize storage
    if (mcc_platform_storage_init() != 0) {
        printf("Failed to initialize storage\n" );
        return;
    }

    // Initialize platform-specific components
    if(mcc_platform_init() != 0) {
        printf("ERROR - platform_init() failed!\n");
        return;
    }
 hum_temp->enable();
    // Print platform information
    mcc_platform_sw_build_info();

    // Print some statistics of the object sizes and their heap memory consumption.
    // NOTE: This *must* be done before creating MbedCloudClient, as the statistic calculation
    // creates and deletes M2MSecurity and M2MDevice singleton objects, which are also used by
    // the MbedCloudClient.
#ifdef MBED_HEAP_STATS_ENABLED
    print_m2mobject_stats();
#endif

    // SimpleClient is used for registering and unregistering resources to a server.
    SimpleM2MClient mbedClient;

    // application_init() runs the following initializations:
    //  1. platform initialization
    //  2. print memory statistics if MBED_HEAP_STATS_ENABLED is defined
    //  3. FCC initialization.
    if (!application_init()) {
        printf("Initialization failed, exiting application!\n");
        return;
    }

    // Save pointer to mbedClient so that other functions can access it.
    client = &mbedClient;

#ifdef MBED_HEAP_STATS_ENABLED
    printf("Client initialized\r\n");
    print_heap_stats();
#endif
#ifdef MBED_STACK_STATS_ENABLED
    print_stack_statistics();
#endif

    // Create resource for button count. Path of this resource will be: 3200/0/5501.
    button_res = mbedClient.add_cloud_resource(3200, 0, 5501, "button_resource", M2MResourceInstance::INTEGER,
                              M2MBase::GET_ALLOWED, 0, true, NULL, (void*)button_notification_status_callback);

    // Create resource for led blinking pattern. Path of this resource will be: 3201/0/5853.
    pattern_res = mbedClient.add_cloud_resource(3201, 0, 5853, "pattern_resource", M2MResourceInstance::STRING,
                               M2MBase::GET_PUT_ALLOWED, "500:500:500:500", false, (void*)pattern_updated, NULL);

	temp_res = mbedClient.add_cloud_resource(3303,0,5700, "temp_resource", M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED, 0, true, NULL, (void*)button_notification_status_callback);

	hum_res = mbedClient.add_cloud_resource(3304,0,5700, "hum_resource", M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED, 0, true, NULL, (void*)button_notification_status_callback);

	cds_res = mbedClient.add_cloud_resource(3301,0,5700, "cds_resouce", M2MResourceInstance::FLOAT, M2MBase::GET_ALLOWED, 0, true, NULL, (void*)button_notification_status_callback);

	led_res = mbedClient.add_cloud_resource(3311,0,5850, "cds_condition", M2MResourceInstance::INTEGER, M2MBase::GET_ALLOWED, 0, true, NULL, (void*)button_notification_status_callback);

    // Create resource for starting the led blinking. Path of this resource will be: 3201/0/5850.
    mbedClient.add_cloud_resource(3201, 0, 5850, "blink_resource", M2MResourceInstance::STRING,
                             M2MBase::POST_ALLOWED, "", false, (void*)blink_callback, NULL);

    // Create resource for unregistering the device. Path of this resource will be: 5000/0/1.
    mbedClient.add_cloud_resource(5000, 0, 1, "unregister", M2MResourceInstance::STRING,
                 M2MBase::POST_ALLOWED, NULL, false, (void*)unregister, NULL);

    // Create resource for running factory reset for the device. Path of this resource will be: 5000/0/2.
    mbedClient.add_cloud_resource(5000, 0, 2, "factory_reset", M2MResourceInstance::STRING,
                 M2MBase::POST_ALLOWED, NULL, false, (void*)factory_reset, NULL);

    mbedClient.register_and_connect();
    mcc_platform_do_wait(10000);

    // Check if client is registering or registered, if true sleep and repeat.
    while (mbedClient.is_register_called()) {
        static int button_count = 0;
        mcc_platform_do_wait(1000);
        if (mcc_platform_button_clicked()) {
            button_res->set_value(++button_count);
        }
		hum_temp->get_temperature(&value1);
		if(old_val1 != value1)
		{
			old_val1=value1;
			printf("HTS221: [temp] %7s C\r\n", print_double(buffer1, value1));
			temp_res -> set_value(value1);
		}
		
		hum_temp->get_humidity(&value2);
		if(old_val2 != value2) {
			old_val2=value2;
			printf("HTS221: [hum] %7s%%\r\n", print_double(buffer2, value2));
			hum_res -> set_value(value2);
		}

		cdsVal=CDS.read();
		cds_res -> set_value(cdsVal);
		printf("CDS: [cds value] %7s\r\n", print_double(buffer3, cdsVal));

		sensorReading=Pressure.read();
		printf("Pressure: %7s\r\n", print_double(buffer4, sensorReading)); 


    }

    // Client unregistered, exit program.
}
