#include <SPI.h>
#include <AS5050.h>

#define CHIP_SELECT_PIN 10
#define CLOCK_DIVIDER_VALUE SPI_CLOCK_DIV64

AS5050 mag(CHIP_SELECT_PIN, CLOCK_DIVIDER_VALUE);

static constexpr uint32_t PUBLISH_PERIOD_US = 100000;  // 10 Hz

uint32_t next_pub_us = 0;
bool have_prev = false;
float prev_angle_rad = 0.0f;
uint32_t prev_t_us = 0;

void setup() {
  Serial.begin(115200);
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
  next_pub_us = micros();
}

void loop() {
  // Keep library tracking rotations by polling frequently:
  (void)mag.angle();

  const uint32_t t_us = micros();
  if ((int32_t)(t_us - next_pub_us) < 0) return;
  next_pub_us += PUBLISH_PERIOD_US;

  const float angle_rad = mag.angleRad();  // treat as continuous

  float omega = 0.0f;
  if (have_prev) {
    const uint32_t dt_us = t_us - prev_t_us;
    if (dt_us > 0) {
      const float dt_s = dt_us * 1e-6f;
      omega = (angle_rad - prev_angle_rad) / dt_s;  // no unwrap
    }
  } else {
    have_prev = true;
  }

  prev_angle_rad = angle_rad;
  prev_t_us = t_us;

  float rps = omega / TWO_PI;
  float rpm = rps * 60.0f;

  Serial.print(t_us);
  Serial.print(',');
  Serial.print(angle_rad);
  Serial.print(',');
  Serial.print(rps, 6);
  Serial.print(',');
  Serial.print(rpm, 2);
  Serial.print(',');
  Serial.println(omega, 6);
}
