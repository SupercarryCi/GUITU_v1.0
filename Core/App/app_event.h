#ifndef APP_EVENT_H
#define APP_EVENT_H

#define SYS_EVT_INIT_DONE          (1UL << 0)
#define SYS_EVT_INIT_FAILED        (1UL << 1)
#define SYS_EVT_GYRO_UPDATED       (1UL << 2)
#define SYS_EVT_NAV_UPDATED        (1UL << 3)
#define SYS_EVT_ADC_UPDATED        (1UL << 4)
#define SYS_EVT_SPO2_UPDATED       (1UL << 5)
#define SYS_EVT_UI_COMMAND         (1UL << 6)
#define SYS_EVT_LORA_RX            (1UL << 7)
#define SYS_EVT_LORA_TX_DONE       (1UL << 8)
#define SYS_EVT_RETURN_ACTIVE      (1UL << 9)
#define SYS_EVT_RETURN_DONE        (1UL << 10)
#define SYS_EVT_RETURN_FAULT       (1UL << 11)

#endif
