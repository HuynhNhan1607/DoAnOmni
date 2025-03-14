#define REINIT_TIME 2500
// BNO polling period.
#define BNO_POLLING_MS 100

void ndof_task(void *pvParameters);
void reinit_sensor(void *pvParameters);

void bno055_start(void);
