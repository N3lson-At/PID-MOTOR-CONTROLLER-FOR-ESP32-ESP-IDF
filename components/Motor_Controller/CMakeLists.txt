idf_component_register(SRCS "Motor_Controller.c"
                    INCLUDE_DIRS "include"
                    REQUIRES 
                        driver
                        esp_timer)